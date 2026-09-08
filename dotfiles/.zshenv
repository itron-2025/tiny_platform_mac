# ~/.zshenv — sourced for EVERY zsh (login/interactive or not), and crucially
# BEFORE /etc/zsh/zshrc. That ordering is the whole point of this file.
#
# Ubuntu's global /etc/zsh/zshrc runs a bare `compinit` (no -u/-i). Because this
# dev container uses `umask 000` (so the host can edit bind-mounted files), some
# fpath dirs become world-writable, and that bare compinit then blocks startup
# with an interactive prompt:
#   "zsh compinit: insecure directories ... continue [y] or abort [n]?"
#
# /etc/zsh/zshrc itself documents the escape hatch: set skip_global_compinit=1
# to suppress its compinit. oh-my-zsh later runs its own `compinit -u` (we also
# set ZSH_DISABLE_COMPFIX=true in .zshrc), so completion still works — minus the
# prompt. This must live in .zshenv; setting it in .zshrc would be too late,
# since the global zshrc has already run by then.
skip_global_compinit=1
