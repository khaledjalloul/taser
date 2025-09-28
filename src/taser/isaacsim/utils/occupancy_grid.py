from typing import TYPE_CHECKING

import numpy as np
import omni

from taser.navigation import OccupancyGrid

if TYPE_CHECKING:
    from isaacsim.asset.gen.omap.bindings import _omap

CELL_SIZE = 0.1  # Size of each cell in meters


def set_up_occupancy_grid_generator() -> "_omap.Generator":
    from isaacsim.asset.gen.omap.bindings import _omap

    """
    Set up the occupancy grid generator.
    Returns:
        _omap.Generator: The occupancy grid generator instance.
    """
    physx = omni.physx.acquire_physx_interface()
    stage_id = omni.usd.get_context().get_stage_id()

    generator = _omap.Generator(physx, stage_id)

    # Configure occupancy map: Cell size (m), occupied value, unoccupied value, unknown value
    generator.update_settings(CELL_SIZE, 1, 0, -1)

    # Set the origin location (should not be inside a prim) and the min and max bounds respectively
    generator.set_transform((-5, -5, 0.2), (-0.1, -0.1, 0.0), (10, 10, 0.0))

    generator.generate2d()
    return generator


def get_occupancy_grid(generator: "_omap.Generator", plot: bool) -> np.ndarray:
    generator.generate2d()
    dims = generator.get_dimensions()
    buffer = generator.get_buffer()

    occupancy_grid = np.array(buffer).reshape(dims[1], dims[0])
    # Mark unknown cells as occupied
    occupancy_grid = np.where(occupancy_grid == -1, 1, occupancy_grid)

    min_bound = generator.get_min_bound()
    max_bound = generator.get_max_bound()

    occupancy_grid = OccupancyGrid(
        workspace=(min_bound[0], max_bound[0], min_bound[1], max_bound[1]),
        cellsize=CELL_SIZE,
        grid=occupancy_grid,
    )

    if plot:
        occupancy_grid.plot()

    return occupancy_grid
