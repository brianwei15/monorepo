from sim.world_gen import WorldNode
from sim.world_gen.entity import Entity
from typing import Optional
import rclpy
from ros_gz_interfaces.srv import SpawnEntity
import sys
import json


class SwarmWorldNode(WorldNode):
    def __init__(
        self,
        template_world: str,
        physics: Optional[dict] = None,
        output_filename: Optional[str] = None,
        seed: Optional[int] = None,
        **kwargs,
    ):
        super().__init__(
            competition_name="swarm",
            output_filename=output_filename,
            seed=seed,
            **kwargs,
        )
        self.world_name = template_world
        # defaults to 0.6 if not provided
        self.physics = physics
        self.instantiate_static_world(
            template_world_path=template_world, physics=physics
        )

    def generate_world(self):
        heat_source = Entity(
            name="heat_source_0",
            path_to_sdf="~/.simulation-gazebo/models/heat_source/model.sdf",
            position=(0.2, 0.0, 0.1),
            rpy=(0.0, 0.0, 0.0),
            world=self.world_name,
        )
        req = SpawnEntity.Request()
        req.entity_factory = heat_source.to_entity_factory_msg()
        self.spawn_entity_client.call_async(req)

        return super().generate_world()


def main(args=None):
    rclpy.init(args=args)
    node = SwarmWorldNode(**json.loads(sys.argv[1]))
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
    rclpy.shutdown()
