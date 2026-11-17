#!/usr/bin/env python3
"""
Upload survey photos to WebODM, wait for processing, download the results.

    python webodm_upload.py PHOTO_FOLDER [--url http://localhost:8000] [--user admin]
                            [--project-name "Drone survey"] [--download orthophoto.tif dsm.tif all.zip]

The password is read from the WEBODM_PASSWORD environment variable, or asked
for interactively (never put it on the command line). geo.txt from the camera
script is uploaded with the photos, so WebODM knows where each photo was taken.

Uses the WebODM REST API: token auth -> project -> partial task ->
upload in batches -> commit -> poll status -> download assets.
"""
import argparse
import getpass
import glob
import json
import os
import sys
import time

import requests

STATUS = {10: "QUEUED", 20: "RUNNING", 30: "FAILED", 40: "COMPLETED", 50: "CANCELED"}
IMAGE_EXT = (".jpg", ".jpeg", ".png", ".tif", ".tiff")
DEFAULT_OPTIONS = [
    {"name": "auto-boundary", "value": True},
    {"name": "dsm", "value": True},
    {"name": "mesh-octree-depth", "value": 11},
]


class WebODM:
    def __init__(self, url, user, password):
        self.url = url.rstrip("/")
        r = requests.post(f"{self.url}/api/token-auth/", data={"username": user, "password": password}, timeout=30)
        if r.status_code != 200:
            raise SystemExit(f"Login failed ({r.status_code}): {r.text[:200]}")
        self.headers = {"Authorization": f"JWT {r.json()['token']}"}

    def _req(self, method, path, **kw):
        kw.setdefault("timeout", 120)
        r = requests.request(method, f"{self.url}{path}", headers=self.headers, **kw)
        if r.status_code >= 400:
            raise SystemExit(f"{method} {path} failed ({r.status_code}): {r.text[:300]}")
        return r

    def project(self, project_id, name):
        if project_id:
            return self._req("GET", f"/api/projects/{project_id}/").json()["id"]
        for p in self._req("GET", "/api/projects/").json():
            if p["name"] == name:
                return p["id"]
        return self._req("POST", "/api/projects/", data={"name": name}).json()["id"]

    def create_task(self, pid, name, options):
        data = {"name": name, "options": json.dumps(options), "partial": "true"}
        return self._req("POST", f"/api/projects/{pid}/tasks/", data=data).json()["id"]

    def upload(self, pid, tid, paths, batch=20):
        for i in range(0, len(paths), batch):
            chunk = paths[i:i + batch]
            handles = [open(p, "rb") for p in chunk]
            try:
                files = [("images", (os.path.basename(p), h)) for p, h in zip(chunk, handles)]
                self._req("POST", f"/api/projects/{pid}/tasks/{tid}/upload/", files=files, timeout=600)
            finally:
                for h in handles:
                    h.close()
            print(f"  uploaded {min(i + batch, len(paths))}/{len(paths)}")
        self._req("POST", f"/api/projects/{pid}/tasks/{tid}/commit/")

    def task(self, pid, tid):
        return self._req("GET", f"/api/projects/{pid}/tasks/{tid}/").json()

    def download(self, pid, tid, asset, folder):
        path = os.path.join(folder, asset)
        r = self._req("GET", f"/api/projects/{pid}/tasks/{tid}/download/{asset}", stream=True, timeout=600)
        with open(path + ".part", "wb") as f:
            for chunk in r.iter_content(1 << 20):
                f.write(chunk)
        os.replace(path + ".part", path)
        return path


def main():
    p = argparse.ArgumentParser(description="Process drone photos with WebODM")
    p.add_argument("folder", help="folder with the photos (and geo.txt)")
    p.add_argument("--url", default=os.environ.get("WEBODM_URL", "http://localhost:8000"))
    p.add_argument("--user", default=os.environ.get("WEBODM_USER", "admin"))
    p.add_argument("--project-id", type=int)
    p.add_argument("--project-name", default="Drone survey")
    p.add_argument("--task-name", default=None)
    p.add_argument("--options", help='ODM options as JSON, e.g. \'[{"name":"fast-orthophoto","value":true}]\'')
    p.add_argument("--download", nargs="*", default=["orthophoto.tif", "dsm.tif", "all.zip"],
                   help="assets to download when done (e.g. orthophoto.tif dsm.tif textured_model.zip all.zip)")
    p.add_argument("--out", help="where to save results (default: <folder>/webodm_results)")
    args = p.parse_args()

    photos = sorted(f for f in glob.glob(os.path.join(args.folder, "*")) if f.lower().endswith(IMAGE_EXT))
    if len(photos) < 5:
        raise SystemExit(f"Found {len(photos)} photos in {args.folder} - WebODM needs at least 5 (ideally 50+)")
    extra = [f for f in (os.path.join(args.folder, "geo.txt"), os.path.join(args.folder, "gcp_list.txt"))
             if os.path.exists(f)]

    password = os.environ.get("WEBODM_PASSWORD") or getpass.getpass(f"WebODM password for {args.user}: ")
    odm = WebODM(args.url, args.user, password)
    options = json.loads(args.options) if args.options else DEFAULT_OPTIONS

    pid = odm.project(args.project_id, args.project_name)
    name = args.task_name or os.path.basename(os.path.abspath(args.folder))
    tid = odm.create_task(pid, name, options)
    print(f"Project {pid}, task {tid}: uploading {len(photos)} photos" +
          (f" + {', '.join(os.path.basename(e) for e in extra)}" if extra else ""))
    odm.upload(pid, tid, photos + extra)

    print("Processing (this can take a long time)...")
    last = None
    while True:
        t = odm.task(pid, tid)
        status = t.get("status")
        progress = t.get("running_progress") or 0
        line = f"  {STATUS.get(status, 'UPLOADING' if status is None else status)} {progress * 100:.0f}%"
        if line != last:
            print(line)
            last = line
        if status == 40:
            break
        if status in (30, 50):
            raise SystemExit(f"Task {STATUS[status]}: {t.get('last_error') or 'see WebODM task console'}")
        time.sleep(10)

    out = args.out or os.path.join(args.folder, "webodm_results")
    os.makedirs(out, exist_ok=True)
    available = set(t.get("available_assets") or [])
    for asset in args.download:
        if available and asset not in available:
            print(f"  {asset}: not produced by this task (available: {', '.join(sorted(available))})")
            continue
        print(f"  downloaded {odm.download(pid, tid, asset, out)}")
    print(f"Done. Open the 3D model in WebODM: {odm.url}/dashboard/")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit("Cancelled")
