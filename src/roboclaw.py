import asyncio
import time
import math
from logging import Logger
from dataclasses import dataclass
from typing import (Any, ClassVar, Dict, Final, Mapping, Optional, Sequence,
                    Tuple)

from typing_extensions import Self
from viam.components.motor import *
from viam.errors import *
from viam.logging import getLogger
from viam.module.module import Module
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName
from viam.resource.base import ResourceBase
# from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from src.roboclaw_3 import Roboclaw as OfficialRoboclaw

# Note that this maxRPM value was determined through very limited testing.
max_rpm = 250
minutes_to_ms = 60000
valid_baud_rates = {460800, 230400, 115200, 57600, 38400, 19200, 9600, 2400}
pwm_scale_constant = 32767

class ViamRoboclaw(Motor, Reconfigurable):
    MODEL: ClassVar[Model] = Model(
        ModelFamily("martha", "basicmicro"), "roboclaw"
    )
    address: int
    ticks_per_rotation: float
    motor_channel: int
    baud_rate: int
    serial_path: str
    roboclaw: OfficialRoboclaw
    logger: Logger

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        """This method creates a new instance of this vision service.
        The default implementation sets the name from the `config` parameter and then calls `reconfigure`.

        Args:
            config (ComponentConfig): The configuration for this resource
            dependencies (Mapping[ResourceName, ResourceBase]): The dependencies (both implicit and explicit)

        Returns:
            Self: The resource
        """

        self = cls(config.name)
        cls.validate_config(config)
        self.logger = getLogger("roboclaw")
        self.reconfigure(config, dependencies)

        return self

    @classmethod
    def validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        """This method allows you to validate the configuration object received from the machine,
        as well as to return any implicit dependencies based on that `config`.

        Args:
            config (ComponentConfig): The configuration for this resource

        Returns:
            Sequence[str]: A list of implicit dependencies
        """
        if config.attributes.fields["motor_channel"].number_value < 1 or config.attributes.fields["motor_channel"].number_value > 2:
            raise ValidationError(f'roboclaw motor channel has to be 1 or 2, but is {config.attributes.fields["motor_channel"].number_value}')
            
        if config.attributes.fields["serial_path"].string_value == "":
            raise ValidationError("Error validating, missing required field: 'serial_path'")
        
        if config.attributes.fields["address"].number_value != 0 and (config.attributes.fields["address"].number_value < 128 
            or config.attributes.fields["address"].number_value > 135):
            raise ValidationError("serial address must be between 128 and 135")
        
        if config.attributes.fields["ticks_per_rotation"].number_value < 0:
            raise ValidationError("ticks per rotation must be a positive number")
        
        if not validate_baud_rates(config.attributes.fields["serial_baud_rate"].number_value):
            raise ValidationError(f"baud rate invalid, must be one of these values {valid_baud_rates}")
        
        return []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        """This method allows you to dynamically update your service when it receives a new `config` object.

        Args:
            config (ComponentConfig): The new configuration
            dependencies (Mapping[ResourceName, ResourceBase]): Any dependencies (both implicit and explicit)
        """

        self.serial_path = config.attributes.fields["serial_path"].string_value
        self.ticks_per_rotation = config.attributes.fields["ticks_per_rotation"].number_value
        self.baud_rate = config.attributes.fields["serial_baud_rate"].number_value
        self.motor_channel = config.attributes.fields["motor_channel"].number_value
        self.address = (int(config.attributes.fields["address"].number_value))
        if self.address == 0:
            self.address = 128

        self.roboclaw = OfficialRoboclaw(self.serial_path, self.baud_rate)
        self.roboclaw.Open()

    async def get_position(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargsf
    ) -> float:
        ticks = 0
        if self.motor_channel == 1:
            ticks = self.roboclaw.ReadEncM1(self.address)
        elif self.motor_channel == 2:
            ticks = self.roboclaw.ReadEncM2(self.address)
        return ticks / self.ticks_per_rotation

    async def get_properties(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Motor.Properties:
        props = Motor.Properties
        if self.ticks_per_rotation != 0:
            props.position_reporting = True
        return props

    async def go_for(
        self,
        rpm: float,
        revolutions: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        if revolutions == 0:
            raise ViamError(f"Cannot move motor for 0 revolutions")
        
        warning = check_speed(rpm, max_rpm)
        if warning != "":
            self.logger.warning(warning)

        # If no encoders are present, distance traveled is estimated based on max rpm
        if self.ticks_per_rotation == 0:
            if abs(rpm) > max_rpm:
                rpm = min(rpm, max_rpm)
                rpm = max(rpm, -max_rpm)
            power_pct, wait_dur = go_for_math(rpm, revolutions)
            self.logger.info("Distance traveled is a time based estimation with max rpm = 250. For increased accuracy, connect encoders")
            await self.set_power(power_pct)
            time.sleep(float(wait_dur)/1000.0)
            return await self.stop()

        # If encoders are present, use Roboclaw's SpeedDistance function
        ticks = revolutions * self.ticks_per_rotation
        ticks_per_second = rpm * self.ticks_per_rotation / 60.0
        if self.motor_channel == 1:
            self.roboclaw.SpeedDistanceM1(self.address, int(ticks_per_second), int(ticks), 1)
        elif self.motor_channel == 2:
            self.roboclaw.SpeedDistanceM2(self.address, int(ticks_per_second), int(ticks), 1)

        # wait until the GoFor movement is complete
        while await self.is_moving():
            continue

        return await self.stop()

    async def go_to(
        self,
        rpm: float,
        position_revolutions: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        if self.ticks_per_rotation == 0:
            raise ViamError("roboclaw needs an encoder connected to use GoTo")
        
        rpm = abs(rpm)
        pos = await self.get_position()
        return await self.go_for(rpm, position_revolutions-pos, extra)

    async def is_moving(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> bool:
        move, _ = await self.is_powered()
        return move

    async def is_powered(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Tuple[bool, float]:
        _, pwm1, pwm2 = self.roboclaw.ReadPWMs(self.address)
        if self.motor_channel == 1:
            return pwm1 != 0, float(pwm1)/pwm_scale_constant
        if self.motor_channel == 2:
            return pwm2 != 0, float(pwm2)/pwm_scale_constant
        return False, 0.0

    async def reset_zero_position(
        self,
        offset: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        # if no encoder is provided, position will always return as 0
        if self.ticks_per_rotation == 0:
            raise NotSupportedError("to use reset_zero_position, add an encoder to roboclaw motor")
        
        new_ticks = -1 * offset * self.ticks_per_rotation
        if self.motor_channel == 1:
            return self.roboclaw.SetEncM1(self.address, new_ticks)
        elif self.motor_channel == 2:
            return self.roboclaw.SetEncM2(self.address, new_ticks)

    async def set_power(
        self,
        power: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        if power > 1:
            power = 1
        elif power < -1:
            power = -1

        if self.motor_channel == 1:
            return self.roboclaw.DutyM1(self.address, int(power*pwm_scale_constant))
        elif self.motor_channel == 2:
            return self.roboclaw.DutyM2(self.address, int(power*pwm_scale_constant))

    async def set_rpm(
        self,
        rpm: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        warning = check_speed(rpm, max_rpm)
        if warning != "":
            self.logger.warning(warning)
        
        # if no encoders are connected, estiimate speed using max rpm
        if self.ticks_per_rotation == 0:
            self.logger.warning(f"speed is an estimation based on the max rpm ({max_rpm}), but speed and power"+
                                " do not have a linear relationship; for increased accuracey, connect encoders")
            if abs(rpm) > max_rpm:
                rpm = min(rpm, max_rpm)
                rpm = max(rpm, -max_rpm)
            power_pct = rpm / max_rpm
            return await self.set_power(power_pct)
        
        # if encoder are connected, convert rpm to ticks per second for roboclaw commands
        ticks_per_second = rpm * self.ticks_per_rotation / 60.0
        dist = math.inf
        if rpm < 0:
            dist = -math.inf

        if self.motor_channel == 1:
            return self.roboclaw.SpeedDistanceM1(self.address, ticks_per_second, dist, 1)
        elif self.motor_channel == 2:
            return self.roboclaw.SpeedDistanceM2(self.address, ticks_per_second, dist, 1)

    async def stop(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        return await self.set_power(0)

def validate_baud_rates(baudRate) -> bool:
    is_valid = False
    for val in valid_baud_rates:
        if val == baudRate:
            is_valid = True
    return is_valid

def check_speed(rpm, max) -> str:
	speed = abs(rpm)
	if speed < 0.1:
		return "motor speed is nearly 0 rev_per_min", ViamError(f"Cannot move motor at an RPM that is nearly 0")
	elif max > 0 and speed > max-0.1:
		return "motor speed is nearly the max rev_per_min {max}"
	else:
		return ""
    
def go_for_math(rpm, revolutions):
    dir = 1
    if rpm * revolutions < 0:
        dir = -1
    
    power_pct = abs(rpm) / max_rpm * dir
    wait_dur = abs(revolutions/rpm) * minutes_to_ms
    return power_pct, wait_dur

async def main():
    """
    This function creates and starts a new module, after adding all desired
    resource models. Resource creators must be registered to the resource
    registry before the module adds the resource model.
    """
    Registry.register_resource_creator(
        Motor.SUBTYPE,
        ViamRoboclaw.MODEL,
        ResourceCreatorRegistration(ViamRoboclaw.new, ViamRoboclaw.validate_config))
    module = Module.from_args()

    module.add_model_from_registry(Motor.SUBTYPE, ViamRoboclaw.MODEL)
    await module.start()

if __name__ == "__main__":
    asyncio.run(main())
