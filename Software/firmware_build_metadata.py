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
if profile == "staging" or profile.startswith("staging-") or profile == "hat-validation-16mb":
    env.Append(LINKFLAGS=["-Wl,--wrap=log_printf"])
    variant = "16mb" if profile == "hat-validation-16mb" else (profile.split("-", 1)[1] if "-" in profile else "16mb")
    env.Append(CPPDEFINES=[
        "RAD_HIL_ARDUINO_HTTP",
        ("RAD_HIL_PRODUCT", env.StringifyMacro("ossm")),
        ("RAD_HIL_VARIANT", env.StringifyMacro(variant)),
    ])
    if profile != "hat-validation-16mb":
        # Existing shared-PC HIL runs retain a single master. The explicit
        # onboard-guard profile owns the bus and starts without auto-homing.
        env.Append(CPPDEFINES=["OSSM_HAT_LISTEN_ONLY"])

# Development screen rows and SDK logs share the serial writer.
if env.subst("$PIOENV").startswith("development") and "espidf" in env.get("PIOFRAMEWORK", []):
    env.Append(LINKFLAGS=["-Wl,--wrap=log_printf"])

# Model names describe this image's target, separate from measured physical RAM.
import sys
sys.path.insert(0, env.subst("$PROJECT_DIR"))
from serial_identity_build import install as install_serial_identity
install_serial_identity(env, "OSSM")
