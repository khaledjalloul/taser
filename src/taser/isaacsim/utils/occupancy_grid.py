from typing import TYPE_CHECKING

import numpy as np
import omni

from taser.common.datatypes import Workspace
from taser.navigation import OccupancyGrid

if TYPE_CHECKING:
    from isaacsim.asset.gen.omap.bindings import _omap

CELL_SIZE = 0.1  # Size of each cell in meters


class IsaacSimOccupancyGridGenerator:
    def setup(self) -> "_omap.Generator":
        from isaacsim.asset.gen.omap.bindings import _omap

        """
        Set up the occupancy grid generator.
        Returns:
            _omap.Generator: The occupancy grid generator instance.
        """
        physx = omni.physx.acquire_physx_interface()
        stage_id = omni.usd.get_context().get_stage_id()

        self.generator = _omap.Generator(physx, stage_id)

        # Configure occupancy map: Cell size (m), occupied value, unoccupied value, unknown value
        self.generator.update_settings(CELL_SIZE, 1, 0, -1)

        # Set the origin location (should not be inside a prim) and the min and max bounds respectively
        self.generator.set_transform((-5, -5, 0.2), (-0.1, -0.1, 0.0), (10, 10, 0.0))

        self.generator.generate2d()

        min_bound = self.generator.get_min_bound()
        max_bound = self.generator.get_max_bound()

        self.workspace = Workspace(
            x_min=min_bound[0],
            x_max=max_bound[0],
            y_min=min_bound[1],
            y_max=max_bound[1],
        )

    def get_occupancy_grid(self) -> OccupancyGrid:
        self.generator.generate2d()
        dims = self.generator.get_dimensions()
        buffer = self.generator.get_buffer()

        occupancy_grid = np.array(buffer).reshape(dims[1], dims[0])

        # Flip horizontally to match the world frame
        occupancy_grid = np.fliplr(occupancy_grid)

        # Mark unknown cells as occupied
        occupancy_grid = np.where(occupancy_grid == -1, 1, occupancy_grid)

        occupancy_grid = OccupancyGrid(
            workspace=self.workspace,
            cellsize=CELL_SIZE,
            grid=occupancy_grid,
        )

        return occupancy_grid
