# Helpers for the demo terminals: a prompt + "typed" command, then run it.
source "$HOME/survey_sim/sim_env.sh"
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
export RCUTILS_COLORIZED_OUTPUT=1
export PYTHONUNBUFFERED=1
cd "$HOME/survey_sim"

say() { printf '\n\033[1;36m# %s\033[0m\n' "$*"; }

run() {
  printf '\033[1;32mkali@drone\033[0m:\033[1;34m~/survey\033[0m$ '
  local s="$*" i
  for ((i = 0; i < ${#s}; i++)); do printf '%s' "${s:i:1}"; sleep 0.025; done
  printf '\n'
  eval "$s"
}
