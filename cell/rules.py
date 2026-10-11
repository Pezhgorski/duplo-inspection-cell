"""Deterministic geometry checks: the rules decide shape, the model decides surface."""
from cell.geometry import Part

SIZE_K = 4.0       # tolerance in standard deviations of the good bricks
SIZE_FLOOR = 8.0   # px, minimum tolerance so a very consistent training set is not too strict


def size_reference(parts: list[Part]) -> dict:
    lengths = [p.length for p in parts]
    widths = [p.width for p in parts]
    mean = lambda v: sum(v) / len(v)
    std = lambda v: (sum((x - mean(v)) ** 2 for x in v) / len(v)) ** 0.5
    return {"length_mean": mean(lengths), "length_std": std(lengths),
            "width_mean": mean(widths), "width_std": std(widths)}


def check_size(part: Part, ref: dict) -> list[str]:
    length_tol = max(SIZE_K * ref["length_std"], SIZE_FLOOR)
    width_tol = max(SIZE_K * ref["width_std"], SIZE_FLOOR)
    if abs(part.length - ref["length_mean"]) > length_tol or abs(part.width - ref["width_mean"]) > width_tol:
        return ["WRONG_SIZE"]
    return []
