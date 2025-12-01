from isaacsim.core.utils.extensions import enable_extension

EXTENSIONS = [
    "isaacsim.ros2.bridge",
    "isaacsim.ros2.tf_viewer",
    "isaacsim.ros2.urdf",
    "isaacsim.asset.exporter.urdf",
    "isaacsim.code_editor.vscode",
    "isaacsim.asset.gen.omap",
    "isaacsim.asset.gen.omap.ui",
    "omni.kit.property.physx",
]


def enable_extensions() -> None:
    for ext in EXTENSIONS:
        enable_extension(ext)
