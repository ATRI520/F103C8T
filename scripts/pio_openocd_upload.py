# Post-hook: avoid hardware reset in OpenOCD upload command.
# Some Blue Pill + ST-Link setups cannot toggle NRST, so "reset" fails after a good flash.
# Use "resume" so the CPU actually runs and USART log appears in monitor.

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
            new_flags.append(s.replace("verify reset; shutdown", "verify; resume; shutdown"))
        elif "verify; shutdown" in s:
            new_flags.append(s.replace("verify; shutdown", "verify; resume; shutdown"))
        else:
            new_flags.append(item)
    env.Replace(UPLOADERFLAGS=new_flags)


_patch_openocd_upload_flags()
