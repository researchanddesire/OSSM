Import("env")

import os

env.Append(
    CPPDEFINES=[
        (
            "FIRMWARE_BUILD_SHA",
            env.StringifyMacro(
                os.getenv("RELEASE_BUILD_SHA", os.getenv("GITHUB_SHA", "unknown"))
            ),
        )
    ]
)

# Staging hardware observation uses the ordinary application and normal updates.
profile = env.subst("$PIOENV")
if profile == "staging" or profile.startswith("staging-"):
    env.Append(LINKFLAGS=["-Wl,--wrap=log_printf"])
    variant = profile.split("-", 1)[1] if "-" in profile else "16mb"
    env.Append(CPPDEFINES=[
        "RAD_HIL_ARDUINO_HTTP",
        ("RAD_HIL_PRODUCT", env.StringifyMacro("ossm")),
        ("RAD_HIL_VARIANT", env.StringifyMacro(variant)),
    ])
