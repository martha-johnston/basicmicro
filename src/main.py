import asyncio

from viam.components.motor import Motor
from viam.module.module import Module
from viam.resource.registry import Registry, ResourceCreatorRegistration
from roboclaw import Roboclaw

async def main():
    """
    This function creates and starts a new module, after adding all desired
    resource models. Resource creators must be registered to the resource
    registry before the module adds the resource model.
    """
    Registry.register_resource_creator(
        Motor.SUBTYPE,
        Roboclaw.MODEL,
        ResourceCreatorRegistration(Roboclaw.new, Roboclaw.validate_config))
    module = Module.from_args()

    module.add_model_from_registry(Motor.SUBTYPE, Roboclaw.MODEL)
    await module.start()

if __name__ == "__main__":
    asyncio.run(main())