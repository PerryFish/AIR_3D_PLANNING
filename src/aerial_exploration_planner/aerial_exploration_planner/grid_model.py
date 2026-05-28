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
    corridor_keep_free = set()
    for i in range(spec.x_cells):
        if i % 4 != 0:
            corridor_keep_free.add((i, i))
        if i % 5 != 0:
            corridor_keep_free.add((spec.x_cells - 1 - i, i))
    for iy in range(spec.y_cells):
        for ix in range(spec.x_cells):
            cell = (ix, iy)
            if cell in protected or cell in corridor_keep_free:
                continue
            if (ix + 2 * iy) % 4 in (0, 1):
                occupied.add(cell)
            if len(occupied) >= target:
                return occupied
    for iy in range(spec.y_cells):
        for ix in range(spec.x_cells):
            cell = (ix, iy)
            if cell not in protected and cell not in corridor_keep_free:
                occupied.add(cell)
            if len(occupied) >= target:
                return occupied
    return occupied


def ground_to_occupied_voxels(spec, ground_cells):
    occupied = set()
    for ix, iy in ground_cells:
        height_m = 0.7 + ((ix * 3 + iy * 5) % 5) * 0.45
        for iz in range(spec.z_cells):
            z_center = iz * spec.resolution + spec.resolution * 0.5
            if z_center <= height_m:
                occupied.add((ix, iy, iz))
    return occupied


def grid_to_world(spec, idx):
    return (
        (idx[0] - spec.x_cells / 2) * spec.resolution + spec.resolution * 0.5,
        (idx[1] - spec.y_cells / 2) * spec.resolution + spec.resolution * 0.5,
        idx[2] * spec.resolution + spec.resolution * 0.5,
    )


def world_to_grid(spec, xyz):
    return (
        int(xyz[0] / spec.resolution + spec.x_cells / 2),
        int(xyz[1] / spec.resolution + spec.y_cells / 2),
        int(xyz[2] / spec.resolution),
    )


def in_bounds(spec, idx):
    return 0 <= idx[0] < spec.x_cells and 0 <= idx[1] < spec.y_cells and 0 <= idx[2] < spec.z_cells
