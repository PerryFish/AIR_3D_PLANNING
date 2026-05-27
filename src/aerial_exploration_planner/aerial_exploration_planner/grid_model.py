from dataclasses import dataclass


@dataclass(frozen=True)
class GridSpec:
    x_cells: int
    y_cells: int
    z_cells: int
    resolution: float
    target_ground_ratio: float

    @property
    def total_voxels(self):
        return self.x_cells * self.y_cells * self.z_cells

    @property
    def ground_total_cells(self):
        return self.x_cells * self.y_cells

    @property
    def target_ground_occupied_cells(self):
        return int(round(self.ground_total_cells * self.target_ground_ratio))


def make_dense50_ground_footprint(spec):
    occupied = set()
    target = spec.target_ground_occupied_cells
    protected = {
        (0, 0),
        (1, 0),
        (0, 1),
        (spec.x_cells - 1, spec.y_cells - 1),
        (spec.x_cells - 2, spec.y_cells - 1),
        (spec.x_cells - 1, spec.y_cells - 2),
    }
    for iy in range(spec.y_cells):
        for ix in range(spec.x_cells):
            cell = (ix, iy)
            if cell in protected:
                continue
            if (ix + 2 * iy) % 4 in (0, 1):
                occupied.add(cell)
            if len(occupied) >= target:
                return occupied
    for iy in range(spec.y_cells):
        for ix in range(spec.x_cells):
            cell = (ix, iy)
            if cell not in protected:
                occupied.add(cell)
            if len(occupied) >= target:
                return occupied
    return occupied
