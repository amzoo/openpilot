import math
from dataclasses import dataclass, field


@dataclass
class LeadVehicle:
  glow: list = field(default_factory=list)
  chevron: list = field(default_factory=list)
  chevron_pos: tuple = (0, 0)
  fill_alpha: int = 0


def _round_triangle(vertices, radius, segments_per_corner=8):
  """Generate rounded triangle vertices for draw_triangle_fan."""
  cx = sum(v[0] for v in vertices) / 3
  cy = sum(v[1] for v in vertices) / 3

  perimeter = []
  n = len(vertices)
  for i in range(n):
    p = vertices[i]
    prev_v = vertices[(i - 1) % n]
    next_v = vertices[(i + 1) % n]

    d1x, d1y = prev_v[0] - p[0], prev_v[1] - p[1]
    d2x, d2y = next_v[0] - p[0], next_v[1] - p[1]
    len1 = math.hypot(d1x, d1y)
    len2 = math.hypot(d2x, d2y)
    d1x, d1y = d1x / len1, d1y / len1
    d2x, d2y = d2x / len2, d2y / len2

    dot = max(-1.0, min(1.0, d1x * d2x + d1y * d2y))
    half_angle = math.acos(dot) / 2
    sin_ha = math.sin(half_angle)
    if sin_ha < 1e-6:
      perimeter.append(p)
      continue

    tan_len = radius / math.tan(half_angle)
    bx, by = d1x + d2x, d1y + d2y
    blen = math.hypot(bx, by)
    bx, by = bx / blen, by / blen
    offset = radius / sin_ha
    acx = p[0] + bx * offset
    acy = p[1] + by * offset

    t1x, t1y = p[0] + d1x * tan_len, p[1] + d1y * tan_len
    t2x, t2y = p[0] + d2x * tan_len, p[1] + d2y * tan_len

    a1 = math.atan2(t1y - acy, t1x - acx)
    a2 = math.atan2(t2y - acy, t2x - acx)
    da = a2 - a1
    while da > math.pi:
      da -= 2 * math.pi
    while da < -math.pi:
      da += 2 * math.pi

    for j in range(segments_per_corner + 1):
      t = j / segments_per_corner
      angle = a1 + da * t
      perimeter.append((acx + radius * math.cos(angle), acy + radius * math.sin(angle)))

  perimeter.append(perimeter[0])
  return [(cx, cy)] + perimeter


def _capsule_vertices(p1, p2, radius, segments=8):
  """Generate triangle-fan vertices for a capsule (stadium) shape between two points."""
  dx, dy = p2[0] - p1[0], p2[1] - p1[1]
  length = math.hypot(dx, dy)
  if length < 1e-6:
    return []
  nx, ny = -dy / length, dx / length

  cx = (p1[0] + p2[0]) / 2
  cy = (p1[1] + p2[1]) / 2

  perimeter = []
  # Semicircle around p1
  base_angle = math.atan2(-ny, -nx)
  for j in range(segments + 1):
    a = base_angle + math.pi * j / segments
    perimeter.append((p1[0] + radius * math.cos(a), p1[1] + radius * math.sin(a)))
  # Semicircle around p2
  base_angle = math.atan2(ny, nx)
  for j in range(segments + 1):
    a = base_angle + math.pi * j / segments
    perimeter.append((p2[0] + radius * math.cos(a), p2[1] + radius * math.sin(a)))

  perimeter.append(perimeter[0])
  return [(cx, cy)] + perimeter


def build_chevron(x, y, sz, g_xo, g_yo, style):
  """Build chevron polygons based on style. Returns (glow, chevron, chevron_pos)."""
  glow_tri = [(x + (sz * 1.35) + g_xo, y + sz + g_yo), (x, y - g_yo), (x - (sz * 1.35) - g_xo, y + sz + g_yo)]
  chevron_tri = [(x + (sz * 1.25), y + sz), (x, y), (x - (sz * 1.25), y + sz)]

  if style == 2:  # Open V
    arm_r = sz * 0.15
    glow = [_capsule_vertices(glow_tri[1], glow_tri[0], arm_r), _capsule_vertices(glow_tri[1], glow_tri[2], arm_r)]
    chevron = [_capsule_vertices(chevron_tri[1], chevron_tri[0], arm_r), _capsule_vertices(chevron_tri[1], chevron_tri[2], arm_r)]
  elif style == 1:  # Filled (rounded)
    corner_r = sz * 0.15
    glow = [_round_triangle(glow_tri, corner_r)]
    chevron = [_round_triangle(chevron_tri, corner_r)]
  else:  # Default (sharp)
    glow = [glow_tri]
    chevron = [chevron_tri]

  return glow, chevron, (x, y)
