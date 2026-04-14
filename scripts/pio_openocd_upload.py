# Post-hook: drop hardware reset from OpenOCD's default "program ... verify reset; shutdown".
# Fixes upload returning error 1 when flash+verify succeed but NRST is not wired to ST-Link.
# Keeps generic board (no bluepill "reset_config none") so SWD init stays compatible.

Import("env")


def _patch_openocd_upload_flags():
    uploader = env.get("UPLOADER", "")
    if not uploader or "openocd" not in str(uploader).lower():
        return
    flags = env.get("UPLOADERFLAGS")
    if not flags:
        return
    new_flags = []
    for item in flags:
        s = str(item)
        if "verify reset; shutdown" in s:
            new_flags.append(s.replace("verify reset; shutdown", "verify; shutdown"))
        else:
            new_flags.append(item)
    env.Replace(UPLOADERFLAGS=new_flags)


_patch_openocd_upload_flags()
