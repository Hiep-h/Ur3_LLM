#!/usr/bin/env python3

class ZoneManager:
    def __init__(self, zones: list[str], temp_zone: str = "temp_zone"):
        self.temp_zone = temp_zone
        all_zones = set(zones) | {temp_zone}
        self.occupancy = {z: None for z in all_zones}

    def object_at(self, zone: str):
        return self.occupancy.get(zone)

    def set_object(self, zone: str, obj: str):
        for z, o in self.occupancy.items():
            if o == obj:
                self.occupancy[z] = None
        self.occupancy[zone] = obj

    def clear(self, zone: str):
        self.occupancy[zone] = None

    def free_temp_zone(self) -> str | None:
        return self.temp_zone if self.occupancy[self.temp_zone] is None else None


def expand_plan_with_conflict_resolution(steps: list[dict], zm: ZoneManager) -> list[dict]:
    new_steps = []
    i = 0
    while i < len(steps):
        step = steps[i]
        if (step["skill"] == "pick" and i + 1 < len(steps)
                and steps[i + 1]["skill"] == "place"):
            obj = step["object"]
            place_step = steps[i + 1]
            target_zone = place_step["zone"]
            occupant = zm.object_at(target_zone)

            if occupant is not None and occupant != obj:
                free_zone = zm.free_temp_zone()
                if free_zone is None:
                    raise RuntimeError(
                        f"Không còn chỗ trống để dọn vật '{occupant}' (temp_zone đang bị chiếm)"
                    )
                new_steps.append({"skill": "pick", "object": occupant})
                new_steps.append({"skill": "place", "object": occupant, "zone": free_zone})
                zm.set_object(free_zone, occupant)

            new_steps.append(step)
            new_steps.append(place_step)
            zm.set_object(target_zone, obj)
            i += 2
        else:
            new_steps.append(step)
            if step["skill"] == "place":
                zm.set_object(step["zone"], step["object"])
            i += 1
    return new_steps
