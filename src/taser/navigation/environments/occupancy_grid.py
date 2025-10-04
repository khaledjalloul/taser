import matplotlib.pyplot as plt
import numpy as np

from taser.common.datatypes import Workspace


class OccupancyGrid:
    """
    Custom occupancy grid class for 2D environments.
    Implemented because the one in roboticstoolbox has some issues when cellsize is not 1.
    """

    def __init__(
        self,
        workspace: Workspace,
        cellsize: float,
        grid: np.ndarray = None,
    ):
        self._workspace = workspace
        self._cellsize = cellsize

        if grid is not None:
            self._grid = grid
            x_shape, y_shape = grid.shape
        else:
            x_shape = (
                int((self._workspace.x_max - self._workspace.x_min) // cellsize) + 2
            )
            y_shape = (
                int((self._workspace.y_max - self._workspace.y_min) // cellsize) + 2
            )
            self._grid = np.zeros((y_shape, x_shape))

    def set(self, region: tuple[int, int, int, int], value: float) -> None:
        """Set the value of a region in the occupancy grid.
        Args:
            region (tuple[int, int, int, int]): The region to set in the format (x_min, x_max, y_min, y_max).
            value (float): The value to set the region to (0 for free, 1 for occupied).
        """
        x_min, x_max, y_min, y_max = region

        start_g = self.w2g((x_min, y_min))
        end_g = self.w2g((x_max, y_max))

        self._grid[
            start_g[1] : end_g[1] + 1,
            start_g[0] : end_g[0] + 1,
        ] = value

    def w2g(self, point: tuple[float, float]) -> tuple[int, int]:
        x_w, y_w = point
        x_g = (x_w - self._workspace.x_min) // self._cellsize
        y_g = (y_w - self._workspace.y_min) // self._cellsize
        return (
            int(x_g) + (1 if x_g != 0 else 0),
            int(y_g) + (1 if y_g != 0 else 0),
        )

    def g2w(self, point: tuple[float, float]) -> tuple[float, float]:
        x_g, y_g = point
        x_w = x_g * self._cellsize + self._workspace.x_min
        y_w = y_g * self._cellsize + self._workspace.y_min
        return x_w, y_w

    def plot(self) -> None:
        plt.imshow(
            self._grid, origin="lower", extent=self._workspace.tuple(), cmap="Greys"
        )
        plt.show(block=False)
        plt.pause(0.1)
        plt.clf()

    @property
    def grid(self) -> np.ndarray:
        return self._grid

    @property
    def cellsize(self) -> float:
        return self._cellsize

    @property
    def workspace(self) -> Workspace:
        return self._workspace


if __name__ == "__main__":
    import logging

    import matplotlib.pyplot as plt

    logging.basicConfig(level=logging.INFO, format="")
    logger = logging.getLogger()

    wksp = (-5, 5, -5, 5)
    cellsize = 0.1

    og = OccupancyGrid(workspace=wksp, cellsize=cellsize)
    logger.info("Shape: %s", og.grid.shape)

    logger.info("World to Grid:")
    t1 = og.w2g((wksp[0], wksp[2]))
    logger.info("(%d, %d) -> %s", wksp[0], wksp[2], t1)
    t2 = og.w2g((wksp[0], wksp[3]))
    logger.info("(%d, %d) -> %s", wksp[0], wksp[3], t2)
    t3 = og.w2g((wksp[1], wksp[2]))
    logger.info("(%d, %d) -> %s", wksp[1], wksp[2], t3)
    t4 = og.w2g((wksp[1], wksp[3]))
    logger.info("(%d, %d) -> %s", wksp[1], wksp[3], t4)
    t5 = og.w2g((2, 3))
    logger.info("(2, 3) -> %s", t5)

    logger.info("Grid to World:")
    logger.info("%s -> %s", t1, og.g2w(t1))
    logger.info("%s -> %s", t2, og.g2w(t2))
    logger.info("%s -> %s", t3, og.g2w(t3))
    logger.info("%s -> %s", t4, og.g2w(t4))
    logger.info("%s -> %s", t5, og.g2w(t5))

    # Set some occupied cells
    og.set((1, 1, -1, 4), 1)
    og.set((-4, -2, -2, 2), 1)

    # Visualize
    plt.grid()
    plt.imshow(og.grid, origin="lower", extent=wksp, cmap="Greys")
    plt.show(block=True)
