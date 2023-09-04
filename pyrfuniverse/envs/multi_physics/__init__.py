from pyrfuniverse.envs.multi_physics.kinova_gen2_catching_cloth_env import (
    KinovaGen2CatchingClothEnv,
)
from pyrfuniverse.envs.multi_physics.ur5_water_shooting import UR5WaterShootingEnv
from pyrfuniverse.envs.multi_physics.flexiv_cutting import FlexivCuttingEnv

__all__ = ["KinovaGen2CatchingClothEnv", "UR5WaterShootingEnv", "FlexivCuttingEnv"]

try:
    from gym.envs.registration import register

    register(
        id="KinovaCatchingCloth-v4",
        entry_point="pyrfuniverse.envs.multi_physics:KinovaGen2CatchingClothEnv",
        kwargs={
            "lock_eef_height": True,
            "with_force_zone": True,
            "max_steps": 100,
            # 'executable_file': join_path('KinovaGen2CatchingCloth/RFUniverse.x86_64')
        },
    )

except ImportError:
    print("No gym installed. Please install gym!")
    pass
