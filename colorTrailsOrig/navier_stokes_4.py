from __future__ import annotations

"""Stefan Petrick, 2026.

Real-time fluid simulation sketch for LED-based visuals.

The numerical core is inspired by Jos Stam's "Stable Fluids" paper. It uses a
semi-Lagrangian grid solver because that approach remains stable at interactive
time steps and is therefore well suited for live visual exploration. The goal is
not strict physical accuracy; it is a controllable, performant approximation
that produces convincing motion on a low-resolution LED-style grid.

Several visual controls intentionally sit on top of the base solver. Vorticity
confinement, beat swirl, chroma drift, glow and color grading are separated from
the basic advection/projection loop so their artistic impact can be tuned
without hiding the underlying fluid state.

Prototype for a FastLED fluid simulation.
Artistic direction and visual choices by Stefan Petrick.
Released under the MIT License.
"""

import colorsys
from dataclasses import dataclass
import math

import numpy as np
import pygame


WINDOW_WIDTH = 1260
WINDOW_HEIGHT = 920
DEFAULT_GRID_SIZE = 128
GRID_SIZE = DEFAULT_GRID_SIZE
CELL_PIXELS = 11
# The live grid is laid out by FluidApp. These constants remain as shared layout
# defaults for panel geometry and keep the file compatible with earlier helper
# calculations that still reference the original editor proportions.
SIM_PIXELS = 64 * CELL_PIXELS
SIM_X = 36
SIM_Y = 36
PANEL_X = SIM_X + SIM_PIXELS + 40
PANEL_Y = 40
PANEL_WIDTH = WINDOW_WIDTH - PANEL_X - 32

BG_COLOR = (8, 10, 16)
PANEL_COLOR = (18, 22, 32)
TEXT_COLOR = (235, 238, 245)
SUBTLE_TEXT = (150, 157, 172)
TRACK_COLOR = (26, 45, 86)
FILL_COLOR = (90, 44, 148)
KNOB_COLOR = (116, 56, 186)
BUTTON_COLOR = FILL_COLOR
BUTTON_ACTIVE = KNOB_COLOR
GRID_BORDER = (56, 62, 82)
OBSTACLE_COLOR = (205, 218, 245)
OBSTACLE_OUTLINE = (88, 112, 168)
OBSTACLE_WIDTH = 10
OBSTACLE_THICKNESS = 2


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def smoothstep(value: float) -> float:
    return value * value * (3.0 - 2.0 * value)


@dataclass
class Slider:
    # Small immediate-mode style UI primitive. Label surfaces are cached because
    # pygame text rendering allocates work each frame, and the controls are
    # static unless the label itself changes.
    label: str
    min_value: float
    max_value: float
    value: float
    rect: pygame.Rect
    is_int: bool = False
    dragging: bool = False
    label_surface: pygame.Surface | None = None
    label_rect: pygame.Rect | None = None

    def normalized(self) -> float:
        span = self.max_value - self.min_value
        if span <= 0.0:
            return 0.0
        return (self.value - self.min_value) / span

    def set_from_mouse(self, mouse_x: int) -> None:
        ratio = clamp((mouse_x - self.rect.left) / self.rect.width, 0.0, 1.0)
        value = self.min_value + ratio * (self.max_value - self.min_value)
        self.value = int(round(value)) if self.is_int else value

    def handle_event(self, event: pygame.event.Event) -> bool:
        hitbox = self.rect.inflate(0, 12)
        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1 and hitbox.collidepoint(event.pos):
            self.dragging = True
            self.set_from_mouse(event.pos[0])
            return True
        if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False
        if event.type == pygame.MOUSEMOTION and self.dragging:
            self.set_from_mouse(event.pos[0])
            return True
        return False

    def prepare_label(self, font: pygame.font.Font) -> None:
        self.label_surface = font.render(self.label, True, TEXT_COLOR)
        self.label_rect = self.label_surface.get_rect(midleft=(self.rect.left + 12, self.rect.centery))

    def draw(self, surface: pygame.Surface, font: pygame.font.Font, value_font: pygame.font.Font) -> None:
        pygame.draw.rect(surface, TRACK_COLOR, self.rect, border_radius=6)
        fill_width = max(0, int(self.rect.width * self.normalized()))
        if fill_width > 0:
            pygame.draw.rect(
                surface,
                FILL_COLOR,
                pygame.Rect(self.rect.left, self.rect.top, fill_width, self.rect.height),
                border_radius=6,
            )
        knob_x = self.rect.left + fill_width
        knob_radius = max(5, self.rect.height // 2 + 1)
        pygame.draw.circle(surface, KNOB_COLOR, (knob_x, self.rect.centery), knob_radius)

        value_text = str(int(self.value)) if self.is_int else f"{self.value:.4f}"
        value_surface = value_font.render(value_text, True, TEXT_COLOR)
        value_rect = value_surface.get_rect(midright=(self.rect.right - 12, self.rect.centery))
        if self.label_surface is None or self.label_rect is None:
            self.prepare_label(font)
        surface.blit(self.label_surface, self.label_rect)
        surface.blit(value_surface, value_rect)


@dataclass
class Button:
    # Buttons share the same visual vocabulary as sliders. The UI only needs
    # simple toggle and action controls, so a compact widget class is sufficient.
    label: str
    rect: pygame.Rect
    toggled: bool = False
    text_surface: pygame.Surface | None = None
    text_rect: pygame.Rect | None = None

    def prepare_label(self, font: pygame.font.Font) -> None:
        self.text_surface = font.render(self.label, True, TEXT_COLOR)
        self.text_rect = self.text_surface.get_rect(center=self.rect.center)

    def draw(self, surface: pygame.Surface, font: pygame.font.Font) -> None:
        color = BUTTON_ACTIVE if self.toggled else BUTTON_COLOR
        pygame.draw.rect(surface, color, self.rect, border_radius=8)
        if self.text_surface is None or self.text_rect is None:
            self.prepare_label(font)
        surface.blit(self.text_surface, self.text_rect)

    def hit_test(self, pos: tuple[int, int]) -> bool:
        return self.rect.collidepoint(pos)


class FluidSolver:
    def __init__(self, size: int) -> None:
        self.size = size
        shape = (size + 2, size + 2)
        inner_shape = (size, size)
        # The extra one-cell border on each side makes boundary handling simpler.
        # All simulation work happens in [1:-1, 1:-1].
        self.u = np.zeros(shape, dtype=np.float32)
        self.v = np.zeros(shape, dtype=np.float32)
        self.u_prev = np.zeros(shape, dtype=np.float32)
        self.v_prev = np.zeros(shape, dtype=np.float32)
        self.dye_r = np.zeros(shape, dtype=np.float32)
        self.dye_g = np.zeros(shape, dtype=np.float32)
        self.dye_b = np.zeros(shape, dtype=np.float32)
        self.dye_r_prev = np.zeros(shape, dtype=np.float32)
        self.dye_g_prev = np.zeros(shape, dtype=np.float32)
        self.dye_b_prev = np.zeros(shape, dtype=np.float32)
        # Pressure is not a fully independent simulation layer here. It stores
        # the latest projection solve so the debug view can show what the
        # incompressibility pass is doing.
        self.pressure = np.zeros(shape, dtype=np.float32)
        self._scratch = np.zeros(shape, dtype=np.float32)
        self._grad_x = np.zeros(shape, dtype=np.float32)
        self._grad_y = np.zeros(shape, dtype=np.float32)
        self._splat_cache: dict[int, np.ndarray] = {}
        # Hard-boundary helpers are inactive in the current emitter sketch, but
        # the arrays remain part of the solver so obstacle experiments can be
        # re-enabled without rewriting the boundary path.
        self.obstacle_mask = np.zeros(shape, dtype=bool)
        # Optional soft boundary mask for anti-aliased obstacle edges. It allows
        # boundary cells to attenuate velocity/dye instead of switching directly
        # from fully open to fully solid.
        self.obstacle_soft_mask = np.zeros(shape, dtype=np.float32)
        self.obstacle_bounds = (1, 1, 1, 1)
        self.obstacle_segments: list[tuple[int, int, int, int]] = []
        self.has_obstacle = False
        # These arrays are reused every frame to keep advection cheap.
        self._advect_x = np.zeros(inner_shape, dtype=np.float32)
        self._advect_y = np.zeros(inner_shape, dtype=np.float32)
        self._advect_i0 = np.zeros(inner_shape, dtype=np.int32)
        self._advect_i1 = np.zeros(inner_shape, dtype=np.int32)
        self._advect_j0 = np.zeros(inner_shape, dtype=np.int32)
        self._advect_j1 = np.zeros(inner_shape, dtype=np.int32)
        self._advect_s0 = np.zeros(inner_shape, dtype=np.float32)
        self._advect_s1 = np.zeros(inner_shape, dtype=np.float32)
        self._advect_t0 = np.zeros(inner_shape, dtype=np.float32)
        self._advect_t1 = np.zeros(inner_shape, dtype=np.float32)
        self.ii, self.jj = np.meshgrid(
            np.arange(1, size + 1, dtype=np.float32),
            np.arange(1, size + 1, dtype=np.float32),
            indexing="ij",
        )
        self.fast_diffusion_threshold = 1e-8

    def clear(self) -> None:
        # Reset fields in-place so the existing NumPy buffers can be reused.
        for field in (
            self.u,
            self.v,
            self.u_prev,
            self.v_prev,
            self.dye_r,
            self.dye_g,
            self.dye_b,
            self.dye_r_prev,
            self.dye_g_prev,
            self.dye_b_prev,
            self.pressure,
            self._scratch,
            self._grad_x,
            self._grad_y,
        ):
            field.fill(0.0)
        self.obstacle_mask.fill(False)
        self.obstacle_soft_mask.fill(0.0)
        self.obstacle_segments = []
        self.has_obstacle = False

    def add_dye(self, x: int, y: int, color: tuple[float, float, float], amount: float, radius: float) -> None:
        self._splat(self.dye_r, x, y, color[0] * amount, radius)
        self._splat(self.dye_g, x, y, color[1] * amount, radius)
        self._splat(self.dye_b, x, y, color[2] * amount, radius)

    def add_velocity(self, x: int, y: int, amount_x: float, amount_y: float, radius: float) -> None:
        self._splat(self.u, x, y, amount_x, radius)
        self._splat(self.v, x, y, amount_y, radius)

    def add_dye_float(
        self,
        x: float,
        y: float,
        color: tuple[float, float, float],
        amount: float,
        radius: float,
    ) -> None:
        self._splat_float(self.dye_r, x, y, color[0] * amount, radius)
        self._splat_float(self.dye_g, x, y, color[1] * amount, radius)
        self._splat_float(self.dye_b, x, y, color[2] * amount, radius)

    def add_velocity_float(self, x: float, y: float, amount_x: float, amount_y: float, radius: float) -> None:
        self._splat_float(self.u, x, y, amount_x, radius)
        self._splat_float(self.v, x, y, amount_y, radius)

    def _splat(self, field: np.ndarray, x: int, y: int, amount: float, radius: float) -> None:
        # Integer-centered splats are the fastest source injection path. They
        # match the discrete LED-grid model and can reuse cached radial kernels.
        if radius <= 0.0:
            return
        r = max(1, int(np.ceil(radius)))
        x0 = max(1, x - r)
        x1 = min(self.size, x + r)
        y0 = max(1, y - r)
        y1 = min(self.size, y + r)
        if x0 > x1 or y0 > y1:
            return

        dist2 = self._splat_cache.get(r)
        if dist2 is None:
            # Radius values repeat frequently while tuning. Cache the Gaussian
            # falloff per integer radius to avoid rebuilding the same kernel.
            offsets = np.arange(-r, r + 1, dtype=np.float32)
            dx, dy = np.meshgrid(offsets, offsets, indexing="ij")
            dist2 = dx * dx + dy * dy
            self._splat_cache[r] = dist2

        kernel_x0 = x0 - (x - r)
        kernel_x1 = kernel_x0 + (x1 - x0) + 1
        kernel_y0 = y0 - (y - r)
        kernel_y1 = kernel_y0 + (y1 - y0) + 1
        weights = np.exp(
            -dist2[kernel_x0:kernel_x1, kernel_y0:kernel_y1] / max(1e-5, radius * radius * 0.6),
            dtype=np.float32,
        )
        field[x0 : x1 + 1, y0 : y1 + 1] += amount * weights

    def _splat_float(self, field: np.ndarray, x: float, y: float, amount: float, radius: float) -> None:
        # Float-centered splats reduce visible stepping when emitters move with
        # sub-cell precision. They are more expensive than integer splats
        # because the local kernel depends on the fractional center. For a later
        # constrained C++/FastLED port, this can be quantized again if needed.
        if radius <= 0.0:
            return
        r = max(1, int(math.ceil(radius)))
        x = clamp(x, 1.0, float(self.size))
        y = clamp(y, 1.0, float(self.size))
        x0 = max(1, int(math.floor(x - r)))
        x1 = min(self.size, int(math.ceil(x + r)))
        y0 = max(1, int(math.floor(y - r)))
        y1 = min(self.size, int(math.ceil(y + r)))
        if x0 > x1 or y0 > y1:
            return

        rows = np.arange(x0, x1 + 1, dtype=np.float32) - x
        cols = np.arange(y0, y1 + 1, dtype=np.float32) - y
        dx, dy = np.meshgrid(rows, cols, indexing="ij")
        weights = np.exp(-(dx * dx + dy * dy) / max(1e-5, radius * radius * 0.6), dtype=np.float32)
        field[x0 : x1 + 1, y0 : y1 + 1] += amount * weights

    def step(
        self,
        dt: float,
        viscosity: float,
        diffusion: float,
        iterations: int,
        vorticity: float,
        swirl_amount: float,
        swirl_scale: float,
        swirl_speed: float,
        swirl_time: float,
        chroma_drift: float,
        velocity_dissipation: float,
        dye_dissipation: float,
    ) -> None:
        # Velocity is solved first using the standard Stable Fluids order:
        # 1. optionally diffuse velocity,
        # 2. add forces,
        # 3. project to remove divergence,
        # 4. advect velocity through itself,
        # 5. project again because advection can reintroduce divergence.
        # The second projection is necessary because semi-Lagrangian advection
        # can reintroduce divergence even after the first projection. Keeping
        # the velocity close to divergence-free reduces compressible artifacts.
        viscosity_a = dt * viscosity * self.size * self.size
        if viscosity_a > self.fast_diffusion_threshold:
            np.copyto(self.u_prev, self.u)
            np.copyto(self.v_prev, self.v)
            self.diffuse(1, self.u, self.u_prev, viscosity_a, iterations)
            self.diffuse(2, self.v, self.v_prev, viscosity_a, iterations)
        self.apply_obstacle_boundary()
        self.project(self.u, self.v, self.u_prev, self.v_prev, iterations)

        np.copyto(self.u_prev, self.u)
        np.copyto(self.v_prev, self.v)

        self.advect(1, self.u, self.u_prev, self.u_prev, self.v_prev, dt)
        self.advect(2, self.v, self.v_prev, self.u_prev, self.v_prev, dt)
        self.apply_obstacle_boundary()
        self.project(self.u, self.v, self.u_prev, self.v_prev, iterations)

        if vorticity > 0.0 or swirl_amount > 0.0:
            # Vorticity confinement reinforces curl already present in the
            # velocity field. Curl Pulse adds an external curl-like force field.
            # Both operations modify velocity, so a projection follows to remove
            # any divergence introduced by these artistic force injections.
            if vorticity > 0.0:
                self.apply_vorticity_confinement(dt, vorticity)
            if swirl_amount > 0.0:
                self.apply_curl_pulse(dt, swirl_amount, swirl_scale, swirl_speed, swirl_time)
            self.apply_obstacle_boundary()
            self.project(self.u, self.v, self.u_prev, self.v_prev, iterations)

        diffusion_a = dt * diffusion * self.size * self.size
        # Dye is advanced after velocity so color follows the final projected
        # field for this frame. Diffusion is skipped when effectively zero
        # because the iterative solve would not produce a visible difference.
        dye_fields = (
            (self.dye_r, self.dye_r_prev),
            (self.dye_g, self.dye_g_prev),
            (self.dye_b, self.dye_b_prev),
        )
        for dye, dye_prev in dye_fields:
            if diffusion_a > self.fast_diffusion_threshold:
                np.copyto(dye_prev, dye)
                self.diffuse(0, dye, dye_prev, diffusion_a, iterations)
            np.copyto(dye_prev, dye)

        if chroma_drift > 0.0:
            # Dye-Chroma Drift intentionally separates color transport from the
            # physical velocity solve. Red and blue advect through tiny opposite
            # perpendicular velocity offsets while green uses the unmodified
            # velocity. This creates chromatic edge separation without running
            # three independent fluid simulations.
            inner = np.s_[1:-1, 1:-1]
            speed = self._scratch[inner]
            np.multiply(self.u[inner], self.u[inner], out=speed)
            speed += self.v[inner] * self.v[inner]
            np.sqrt(speed, out=speed)
            speed += 1e-6

            # Convert the UI amount into velocity units. The multiplier is kept
            # small so the separation remains a sub-cell sampling effect rather
            # than tearing the dye channels apart.
            drift_velocity = chroma_drift * 4.0
            drift_u = self._grad_x[inner]
            drift_v = self._grad_y[inner]
            drift_u[:, :] = (-self.v[inner] / speed) * drift_velocity
            drift_v[:, :] = (self.u[inner] / speed) * drift_velocity

            np.copyto(self.u_prev, self.u)
            np.copyto(self.v_prev, self.v)
            self.u_prev[inner] += drift_u
            self.v_prev[inner] += drift_v
            self.set_bnd(1, self.u_prev)
            self.set_bnd(2, self.v_prev)
            self.advect(0, self.dye_r, self.dye_r_prev, self.u_prev, self.v_prev, dt)

            self.advect(0, self.dye_g, self.dye_g_prev, self.u, self.v, dt)

            np.copyto(self.u_prev, self.u)
            np.copyto(self.v_prev, self.v)
            self.u_prev[inner] -= drift_u
            self.v_prev[inner] -= drift_v
            self.set_bnd(1, self.u_prev)
            self.set_bnd(2, self.v_prev)
            self.advect(0, self.dye_b, self.dye_b_prev, self.u_prev, self.v_prev, dt)
        else:
            for dye, dye_prev in dye_fields:
                self.advect(0, dye, dye_prev, self.u, self.v, dt)

        if self.has_obstacle:
            for dye in (self.dye_r, self.dye_g, self.dye_b):
                # If hard geometry is active, clear dye inside solid cells after
                # advection. This prevents backtracing from smearing scalar dye
                # through closed obstacle regions.
                for row0, row1, col0, col1 in self.obstacle_segments:
                    dye[row0 : row1 + 1, col0 : col1 + 1] = 0.0

        if velocity_dissipation < 1.0:
            self.u[1:-1, 1:-1] *= velocity_dissipation
            self.v[1:-1, 1:-1] *= velocity_dissipation
        if dye_dissipation < 1.0:
            self.dye_r[1:-1, 1:-1] *= dye_dissipation
            self.dye_g[1:-1, 1:-1] *= dye_dissipation
            self.dye_b[1:-1, 1:-1] *= dye_dissipation

        np.maximum(self.dye_r[1:-1, 1:-1], 0.0, out=self.dye_r[1:-1, 1:-1])
        np.maximum(self.dye_g[1:-1, 1:-1], 0.0, out=self.dye_g[1:-1, 1:-1])
        np.maximum(self.dye_b[1:-1, 1:-1], 0.0, out=self.dye_b[1:-1, 1:-1])
        self.apply_obstacle_boundary()

    def set_bnd(self, b: int, x: np.ndarray) -> None:
        # b selects which component is mirrored at the wall:
        # 0 = scalar field, 1 = horizontal velocity, 2 = vertical velocity.
        # Scalar fields copy at the border so dye does not vanish instantly.
        # Velocity components flip only when they point through a wall, which
        # approximates a simple no-through-flow boundary.
        x[0, 1:-1] = -x[1, 1:-1] if b == 1 else x[1, 1:-1]
        x[-1, 1:-1] = -x[-2, 1:-1] if b == 1 else x[-2, 1:-1]
        x[1:-1, 0] = -x[1:-1, 1] if b == 2 else x[1:-1, 1]
        x[1:-1, -1] = -x[1:-1, -2] if b == 2 else x[1:-1, -2]

        x[0, 0] = 0.5 * (x[1, 0] + x[0, 1])
        x[0, -1] = 0.5 * (x[1, -1] + x[0, -2])
        x[-1, 0] = 0.5 * (x[-2, 0] + x[-1, 1])
        x[-1, -1] = 0.5 * (x[-2, -1] + x[-1, -2])

    def lin_solve(self, b: int, x: np.ndarray, x0: np.ndarray, a: float, c: float, iterations: int) -> None:
        inv_c = 1.0 / c
        # Classic Gauss-Seidel relaxation. It is simple, deterministic and fast
        # enough for this grid size, making it suitable for interactive tuning.
        for _ in range(iterations):
            x[1:-1, 1:-1] = (
                x0[1:-1, 1:-1]
                + a
                * (
                    x[0:-2, 1:-1]
                    + x[2:, 1:-1]
                    + x[1:-1, 0:-2]
                    + x[1:-1, 2:]
                )
            ) * inv_c
            self.set_bnd(b, x)

    def diffuse(self, b: int, x: np.ndarray, x0: np.ndarray, a: float, iterations: int) -> None:
        if a <= self.fast_diffusion_threshold:
            # Fast path: when diffusion or viscosity is numerically negligible,
            # skip the iterative solve and copy the source field directly.
            np.copyto(x, x0)
            self.set_bnd(b, x)
            return
        self.lin_solve(b, x, x0, a, 1.0 + 4.0 * a, iterations)

    def apply_global_force(self, dt: float, force_u: float, force_v: float) -> None:
        # Constant global force hook. It is currently unused in this sketch, but
        # remains useful for testing directional bias in the velocity field.
        self.u[1:-1, 1:-1] += dt * force_u
        self.v[1:-1, 1:-1] += dt * force_v
        self.set_bnd(1, self.u)
        self.set_bnd(2, self.v)

    def set_obstacle(self, row_center: int, col_center: int, width: int, thickness: int) -> None:
        # Optional split-paddle geometry: two hard blocks with an open slot.
        # The current emitter modes do not use it, but keeping the routine here
        # preserves a tested obstacle path for later solver comparisons.
        half_width = width // 2
        half_thickness = thickness // 2
        row0 = int(clamp(row_center - half_thickness, 1, self.size))
        row1 = int(clamp(row_center + max(0, thickness - half_thickness - 1), 1, self.size))
        col0 = int(clamp(col_center - half_width, 1, self.size))
        col1 = int(clamp(col_center + max(0, width - half_width - 1), 1, self.size))
        self.obstacle_mask.fill(False)
        self.obstacle_soft_mask.fill(0.0)
        gap_width = max(1, int(round(width / 3)))
        solid_total = max(0, width - gap_width)
        left_width = solid_total // 2
        right_width = solid_total - left_width
        gap_start = col0 + left_width
        gap_end = min(col1, gap_start + gap_width - 1)
        self.obstacle_segments = []
        if left_width > 0:
            left_col1 = min(col1, gap_start - 1)
            self.obstacle_mask[row0 : row1 + 1, col0 : left_col1 + 1] = True
            self.obstacle_segments.append((row0, row1, col0, left_col1))
        if right_width > 0:
            right_col0 = max(col0, gap_end + 1)
            self.obstacle_mask[row0 : row1 + 1, right_col0 : col1 + 1] = True
            self.obstacle_segments.append((row0, row1, right_col0, col1))
        inner_row0 = max(1, row0 - 1)
        inner_row1 = min(self.size, row1 + 1)
        if left_width > 0:
            # Soft cells sit on the solid side of the gap. They do not close the
            # opening; they only attenuate the neighboring edge to reduce a hard
            # one-cell discontinuity.
            inner_left_col = gap_start - 1
            self.obstacle_soft_mask[row0 : row1 + 1, inner_left_col] = np.maximum(
                self.obstacle_soft_mask[row0 : row1 + 1, inner_left_col],
                0.18,
            )
            self.obstacle_soft_mask[inner_row0 : inner_row1 + 1, inner_left_col] = np.maximum(
                self.obstacle_soft_mask[inner_row0 : inner_row1 + 1, inner_left_col],
                0.22,
            )
        if right_width > 0:
            inner_right_col = gap_end + 1
            self.obstacle_soft_mask[row0 : row1 + 1, inner_right_col] = np.maximum(
                self.obstacle_soft_mask[row0 : row1 + 1, inner_right_col],
                0.18,
            )
            self.obstacle_soft_mask[inner_row0 : inner_row1 + 1, inner_right_col] = np.maximum(
                self.obstacle_soft_mask[inner_row0 : inner_row1 + 1, inner_right_col],
                0.22,
            )
        self.obstacle_bounds = (row0, row1, col0, col1)
        self.has_obstacle = bool(self.obstacle_segments)
        self.apply_obstacle_boundary()

    def apply_obstacle_boundary(self) -> None:
        if not self.has_obstacle:
            return

        # First apply the hard stop inside solid geometry.
        for row0, row1, col0, col1 in self.obstacle_segments:
            segment_mask = self.obstacle_mask[row0 : row1 + 1, col0 : col1 + 1]
            self.u[row0 : row1 + 1, col0 : col1 + 1][segment_mask] = 0.0
            self.v[row0 : row1 + 1, col0 : col1 + 1][segment_mask] = 0.0
            self.dye_r[row0 : row1 + 1, col0 : col1 + 1][segment_mask] = 0.0
            self.dye_g[row0 : row1 + 1, col0 : col1 + 1][segment_mask] = 0.0
            self.dye_b[row0 : row1 + 1, col0 : col1 + 1][segment_mask] = 0.0
            self.u_prev[row0 : row1 + 1, col0 : col1 + 1][segment_mask] = 0.0
            self.v_prev[row0 : row1 + 1, col0 : col1 + 1][segment_mask] = 0.0
            self.dye_r_prev[row0 : row1 + 1, col0 : col1 + 1][segment_mask] = 0.0
            self.dye_g_prev[row0 : row1 + 1, col0 : col1 + 1][segment_mask] = 0.0
            self.dye_b_prev[row0 : row1 + 1, col0 : col1 + 1][segment_mask] = 0.0

            top = max(1, row0 - 1)
            bottom = min(self.size, row1 + 1)
            left = max(1, col0 - 1)
            right = min(self.size, col1 + 1)

            self.u[top, col0 : col1 + 1] = np.minimum(self.u[top, col0 : col1 + 1], 0.0)
            self.u[bottom, col0 : col1 + 1] = np.maximum(self.u[bottom, col0 : col1 + 1], 0.0)
            self.v[row0 : row1 + 1, left] = np.minimum(self.v[row0 : row1 + 1, left], 0.0)
            self.v[row0 : row1 + 1, right] = np.maximum(self.v[row0 : row1 + 1, right], 0.0)
        row0, row1, col0, col1 = self.obstacle_bounds
        soft = self.obstacle_soft_mask[row0 : row1 + 1, col0 : col1 + 1]
        if np.any(soft > 0.0):
            # Feather inner corners slightly. This reduces visual stair-stepping
            # near the gap while preserving the hard obstacle core.
            inv_soft = 1.0 - soft
            dye_fade = 1.0 - soft * 0.35
            self.u[row0 : row1 + 1, col0 : col1 + 1] *= inv_soft
            self.v[row0 : row1 + 1, col0 : col1 + 1] *= inv_soft
            self.u_prev[row0 : row1 + 1, col0 : col1 + 1] *= inv_soft
            self.v_prev[row0 : row1 + 1, col0 : col1 + 1] *= inv_soft
            self.dye_r[row0 : row1 + 1, col0 : col1 + 1] *= dye_fade
            self.dye_g[row0 : row1 + 1, col0 : col1 + 1] *= dye_fade
            self.dye_b[row0 : row1 + 1, col0 : col1 + 1] *= dye_fade
            self.dye_r_prev[row0 : row1 + 1, col0 : col1 + 1] *= dye_fade
            self.dye_g_prev[row0 : row1 + 1, col0 : col1 + 1] *= dye_fade
            self.dye_b_prev[row0 : row1 + 1, col0 : col1 + 1] *= dye_fade
        self.set_bnd(1, self.u)
        self.set_bnd(2, self.v)

    def advect(self, b: int, d: np.ndarray, d0: np.ndarray, u: np.ndarray, v: np.ndarray, dt: float) -> None:
        dt0 = dt * self.size
        # Semi-Lagrangian backtrace: for each destination cell, sample the
        # previous field at the position reached by tracing backward through the
        # velocity field. This method is numerically stable for interactive time
        # steps, though it introduces some numerical diffusion.
        x = self._advect_x
        y = self._advect_y
        np.subtract(self.ii, dt0 * u[1:-1, 1:-1], out=x)
        np.subtract(self.jj, dt0 * v[1:-1, 1:-1], out=y)
        np.clip(x, 0.5, self.size + 0.5, out=x)
        np.clip(y, 0.5, self.size + 0.5, out=y)

        i0 = self._advect_i0
        i1 = self._advect_i1
        j0 = self._advect_j0
        j1 = self._advect_j1
        np.floor(x, out=x)
        i0[:, :] = x.astype(np.int32)
        i1[:, :] = i0 + 1
        np.subtract(self.ii, dt0 * u[1:-1, 1:-1], out=self._advect_s1)
        np.clip(self._advect_s1, 0.5, self.size + 0.5, out=self._advect_s1)
        np.subtract(self._advect_s1, i0, out=self._advect_s1)
        np.floor(y, out=y)
        j0[:, :] = y.astype(np.int32)
        j1[:, :] = j0 + 1
        np.subtract(self.jj, dt0 * v[1:-1, 1:-1], out=self._advect_t1)
        np.clip(self._advect_t1, 0.5, self.size + 0.5, out=self._advect_t1)
        np.subtract(self._advect_t1, j0, out=self._advect_t1)

        s1 = self._advect_s1
        s0 = self._advect_s0
        t1 = self._advect_t1
        t0 = self._advect_t0
        np.subtract(1.0, s1, out=s0)
        np.subtract(1.0, t1, out=t0)
        d[1:-1, 1:-1] = (
            s0 * (t0 * d0[i0, j0] + t1 * d0[i0, j1])
            + s1 * (t0 * d0[i1, j0] + t1 * d0[i1, j1])
        )
        self.set_bnd(b, d)

    def project(
        self,
        u: np.ndarray,
        v: np.ndarray,
        p: np.ndarray,
        div: np.ndarray,
        iterations: int,
    ) -> None:
        # Projection removes divergence from the velocity field. The divergence
        # field measures local expansion/compression; the pressure solve finds a
        # scalar correction whose gradient can be subtracted from velocity.
        div[1:-1, 1:-1] = -0.5 * (
            u[2:, 1:-1] - u[0:-2, 1:-1] + v[1:-1, 2:] - v[1:-1, 0:-2]
        ) / self.size
        p.fill(0.0)
        self.set_bnd(0, div)
        self.set_bnd(0, p)
        self.lin_solve(0, p, div, 1.0, 4.0, iterations)
        # Keep a copy for the pressure debug view. The projection step itself
        # only needs pressure temporarily, but exposing it helps diagnose how
        # strongly the solver is correcting the field.
        np.copyto(self.pressure, p)

        u[1:-1, 1:-1] -= 0.5 * self.size * (p[2:, 1:-1] - p[0:-2, 1:-1])
        v[1:-1, 1:-1] -= 0.5 * self.size * (p[1:-1, 2:] - p[1:-1, 0:-2])
        self.set_bnd(1, u)
        self.set_bnd(2, v)

    def apply_vorticity_confinement(self, dt: float, strength: float) -> None:
        # Vorticity confinement reinforces existing rotational motion. The
        # gradient of curl magnitude points toward stronger vortices; applying a
        # perpendicular force there restores small-scale motion that advection
        # and diffusion tend to damp away.
        curl = self._scratch
        curl.fill(0.0)
        curl[1:-1, 1:-1] = 0.5 * (
            self.v[2:, 1:-1] - self.v[0:-2, 1:-1] - self.u[1:-1, 2:] + self.u[1:-1, 0:-2]
        )

        magnitude = np.abs(curl)
        grad_x = self._grad_x
        grad_y = self._grad_y
        grad_x.fill(0.0)
        grad_y.fill(0.0)
        grad_x[1:-1, 1:-1] = 0.5 * (magnitude[2:, 1:-1] - magnitude[0:-2, 1:-1])
        grad_y[1:-1, 1:-1] = 0.5 * (magnitude[1:-1, 2:] - magnitude[1:-1, 0:-2])

        norm = np.sqrt(grad_x * grad_x + grad_y * grad_y) + 1e-6
        grad_x /= norm
        grad_y /= norm

        force_x = grad_y * curl * strength
        force_y = -grad_x * curl * strength
        self.u[1:-1, 1:-1] += dt * force_x[1:-1, 1:-1]
        self.v[1:-1, 1:-1] += dt * force_y[1:-1, 1:-1]
        self.set_bnd(1, self.u)
        self.set_bnd(2, self.v)

    def apply_curl_pulse(self, dt: float, amount: float, scale: float, speed: float, time_value: float) -> None:
        # Curl Pulse / Beat Swirl: an animated stream-function force field.
        # Unlike vorticity confinement, this does not depend on existing curl.
        # It creates an external rotational forcing pattern. Because the force
        # is derived from a stream-function-like construction, it primarily
        # bends the flow instead of acting as a uniform translation.
        if amount <= 0.0 or scale <= 0.0:
            return
        row = (self.ii - 0.5) / self.size - 0.5
        col = (self.jj - 0.5) / self.size - 0.5
        k = max(0.1, scale) * math.tau
        t = time_value * speed * math.tau

        # Two sine products with different spatial and temporal frequencies
        # produce a deterministic field that avoids obvious tiling. Existing
        # gradient buffers are reused as force buffers to avoid per-frame
        # allocation in the main simulation loop.
        sin_a = np.sin(col * k + t)
        cos_a = np.cos(col * k + t)
        sin_b = np.sin(row * k * 1.31 - t * 0.73)
        cos_b = np.cos(row * k * 1.31 - t * 0.73)
        sin_c = np.sin(col * k * 0.53 - t * 1.37)
        cos_c = np.cos(col * k * 0.53 - t * 1.37)
        sin_d = np.sin(row * k * 1.91 + t * 0.41)
        cos_d = np.cos(row * k * 1.91 + t * 0.41)

        force_row = self._grad_x[1:-1, 1:-1]
        force_col = self._grad_y[1:-1, 1:-1]
        force_row[:, :] = cos_a * k * sin_b + 0.5 * cos_c * k * 0.53 * sin_d
        force_col[:, :] = -(sin_a * cos_b * k * 1.31 + 0.5 * sin_c * cos_d * k * 1.91)
        pulse = dt * amount * 0.025
        self.u[1:-1, 1:-1] += force_row * pulse
        self.v[1:-1, 1:-1] += force_col * pulse
        self.set_bnd(1, self.u)
        self.set_bnd(2, self.v)


class FluidApp:
    def __init__(self) -> None:
        pygame.init()
        pygame.display.set_caption("projector")
        self.screen = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT))
        self.screen_rect = self.screen.get_rect()
        self.clock = pygame.time.Clock()
        self.font_small = pygame.font.SysFont("Menlo", 14)
        self.font_medium = pygame.font.SysFont("Menlo", 16)
        self.font_large = pygame.font.SysFont("Menlo", 20)
        self.font_status_label = pygame.font.SysFont("Menlo", 11)
        self.font_status_value = pygame.font.SysFont("Menlo", 16)

        margin = 24
        panel_gap = 28
        panel_min_width = 360
        grid_side = min(
            self.screen_rect.height - margin,
            self.screen_rect.width - panel_min_width - panel_gap - margin * 2,
        )
        grid_side = max(320, grid_side)
        self.sim_rect = pygame.Rect(margin, 0, grid_side, grid_side)
        self.panel_rect = pygame.Rect(
            self.sim_rect.right + panel_gap,
            self.sim_rect.top,
            self.screen_rect.right - self.sim_rect.right - panel_gap - margin,
            self.sim_rect.height,
        )
        self.emitter_influence_cache: dict[int, np.ndarray] = {}
        self.emitter_debug_arrows: list[tuple[float, float, float, float, float, tuple[float, float, float]]] = []
        self.allocate_grid_buffers(GRID_SIZE)

        self.paused = False
        self.sim_time = 0.0
        # Orbit phase is integrated over time instead of derived directly from
        # sim_time * slider_value. This keeps emitter positions continuous when
        # the speed slider changes during playback.
        self.jet_orbit_phase_degrees = 0.0
        self.jet_noise_time = 0.0
        # Slider mirrors used by the render path. Storing plain values here
        # keeps rendering code independent from UI widget objects.
        self.color_contrast = 1.18
        self.glow_strength = 0.24
        self.highlight_saturation = 0.22
        self.black_level = 0.035
        self.masked_render_boost = 1.0
        self.emitter_density_for_debug = 120.0
        self.should_render_emitter_influence = False
        self.hard_source_bounds: tuple[int, int, int, int] | None = None
        self.hard_source_color: tuple[float, float, float] = (1.0, 0.0, 0.0)
        self.hard_source_overlays: list[tuple[int, int, int, int, tuple[float, float, float]]] = []
        self.color_mode = "triple"
        self.emitter_motion_mode = "C"
        self.view_modes = (
            "color",
            "masked_render",
            "velocity",
            "vorticity",
            "pressure",
            "divergence",
            "emitter",
            "dye_density",
        )
        # TAB cycles through render and diagnostic views. These views are
        # render-only and must not feed back into the simulation state.
        self.view_labels = {
            "color": "Render View (TAB)",
            "masked_render": "Masked Render (TAB)",
            "velocity": "Velocity Map (TAB)",
            "vorticity": "Vorticity Map (TAB)",
            "pressure": "Pressure Map (TAB)",
            "divergence": "Divergence Map (TAB)",
            "emitter": "Emitter Influence Map (TAB)",
            "dye_density": "Dye Density Map (TAB)",
        }
        # Start with the main render view. Diagnostic views are available via
        # TAB without changing simulation behavior.
        self.view_mode_index = 0
        # Each jet gets a different noise seed. The shared speed slider controls
        # the temporal rate, while the seeds decorrelate their angle changes.
        self.jet_noise_time_offset = np.random.uniform(0.0, 1000.0)
        self.jet_noise_seeds = {
            "ring_0": (11.3, 37.7),
            "ring_1": (23.9, 51.2),
            "ring_2": (41.6, 79.4),
            "ring_3": (67.4, 13.8),
            "center": (89.1, 27.5),
        }
        slider_x = self.panel_rect.left + 18
        slider_width = self.panel_rect.width - 36
        slider_height = 18

        specs = [
            ("dt", "Time Step", 0.0, 0.01, 0.0009, False),
            ("viscosity", "Viscosity", 0.0, 0.01, 0.0000, False),
            ("diffusion", "Dye Diffusion", 0.0, 0.01, 0.0000, False),
            ("velocity_dissipation", "Velocity Fade", 0.90, 1.0, 1.0000, False),
            ("dye_dissipation", "Dye Fade", 0.90, 1.0, 0.9978, False),
            ("color_contrast", "Color Contrast", 0.4, 3.6, 0.9630, False),
            ("black_level", "Black Point", 0.0, 0.24, 0.1830, False),
            ("masked_render_boost", "Masked Boost", 0.0, 4.0, 0.0000, False),
            ("iterations", "Solver Iterations", 1, 40, 23, True),
            ("vorticity", "Vorticity", 0.0, 96.0, 93.6296, False),
            ("swirl_amount", "Swirl Amount", 0.0, 50.0, 8.0, False),
            ("swirl_scale", "Swirl Scale", 0.5, 8.0, 3.0, False),
            ("swirl_speed", "Swirl Speed", 0.0, 3.0, 0.7, False),
            ("chroma_drift", "Chroma Drift", 0.0, 2.0, 0.25, False),
            ("emitter_density", "Emitter Density", 0.0, 1800.0, 350.0000, False),
            ("emitter_upward", "Emitter Force", 0.0, 30.0, 11.9259, False),
            ("emitter_radius", "Emitter Radius", 1.0, 24.0, 24.0000, False),
            ("jet_noise_speed", "Jet Noise Angle Speed", 0.0, 3.0, 1.5000, False),
            ("jet_orbit_speed", "Jet Orbit Speed", -1.25, 1.25, -0.1543, False),
            ("emitter_hue_speed", "Emitter Hue Speed", 0.0, 1.5, 0.9491, False),
            # The visible controls focus on emitters, flow energy and color
            # processing. Obstacle controls are intentionally omitted here.
        ]
        slider_top = self.panel_rect.top + 78
        slider_available = self.panel_rect.bottom - slider_top - slider_height - 8
        slider_gap = min(24, max(slider_height + 3, slider_available // max(1, len(specs) - 1)))
        self.slider_order = [key for key, *_ in specs]
        self.sliders: dict[str, Slider] = {}
        for index, (key, label, low, high, value, is_int) in enumerate(specs):
            top = slider_top + index * slider_gap
            self.sliders[key] = Slider(
                label=label,
                min_value=low,
                max_value=high,
                value=value,
                rect=pygame.Rect(slider_x, top, slider_width, slider_height),
                is_int=is_int,
            )
            self.sliders[key].prepare_label(self.font_medium)

        thumbnail_gap = 8
        thumbnail_top = max(slider.rect.bottom for slider in self.sliders.values()) + 12
        thumbnail_width = (slider_width - thumbnail_gap * 3) // 4
        thumbnail_height = thumbnail_width
        self.thumbnail_rects: dict[str, pygame.Rect] = {}
        # Thumbnails stay square and uncropped so each debug view has the same
        # spatial relationship as the main grid.
        for index, mode in enumerate(self.view_modes):
            col = index % 4
            row = index // 4
            left = slider_x + col * (thumbnail_width + thumbnail_gap)
            top = thumbnail_top + row * (thumbnail_height + thumbnail_gap)
            self.thumbnail_rects[mode] = pygame.Rect(left, top, thumbnail_width, thumbnail_height)

        button_y = self.panel_rect.top + 18
        button_gap = 12
        button_total_width = slider_width - button_gap * 4
        button_width_base, button_width_extra = divmod(button_total_width, 5)
        button_widths = [button_width_base + (1 if index < button_width_extra else 0) for index in range(5)]
        button_x = slider_x
        self.reset_button = Button("Reset", pygame.Rect(button_x, button_y, button_widths[0], 34), toggled=False)
        button_x += button_widths[0] + button_gap
        self.double_button = Button("Double", pygame.Rect(button_x, button_y, button_widths[1], 34), toggled=False)
        button_x += button_widths[1] + button_gap
        self.triple_button = Button("Triple", pygame.Rect(button_x, button_y, button_widths[2], 34), toggled=True)
        button_x += button_widths[2] + button_gap
        self.orange_button = Button("Orange", pygame.Rect(button_x, button_y, button_widths[3], 34), toggled=False)
        button_x += button_widths[3] + button_gap
        self.violett_button = Button("Violett", pygame.Rect(button_x, button_y, button_widths[4], 34), toggled=False)
        mini_gap = 6
        mini_y = self.reset_button.rect.bottom + 6
        mini_width = (self.reset_button.rect.width - mini_gap) // 2
        self.grid64_button = Button(
            "64",
            pygame.Rect(self.reset_button.rect.left, mini_y, mini_width, 18),
            toggled=False,
        )
        self.grid128_button = Button(
            "128",
            pygame.Rect(
                self.grid64_button.rect.right + mini_gap,
                mini_y,
                self.reset_button.rect.width - mini_width - mini_gap,
                18,
            ),
            toggled=True,
        )
        low_res_mini_y = self.triple_button.rect.bottom + 6
        low_res_mini_width = (self.triple_button.rect.width - mini_gap) // 2
        self.grid16_button = Button(
            "16",
            pygame.Rect(self.triple_button.rect.left, low_res_mini_y, low_res_mini_width, 18),
            toggled=False,
        )
        self.grid32_button = Button(
            "32",
            pygame.Rect(
                self.grid16_button.rect.right + mini_gap,
                low_res_mini_y,
                self.triple_button.rect.width - low_res_mini_width - mini_gap,
                18,
            ),
            toggled=False,
        )
        motion_mini_y = self.double_button.rect.bottom + 6
        motion_mini_width = (self.double_button.rect.width - mini_gap * 3) // 4
        self.motion_a_button = Button(
            "A",
            pygame.Rect(self.double_button.rect.left, motion_mini_y, motion_mini_width, 18),
            toggled=False,
        )
        self.motion_b_button = Button(
            "B",
            pygame.Rect(
                self.motion_a_button.rect.right + mini_gap,
                motion_mini_y,
                motion_mini_width,
                18,
            ),
            toggled=False,
        )
        self.motion_c_button = Button(
            "C",
            pygame.Rect(
                self.motion_b_button.rect.right + mini_gap,
                motion_mini_y,
                motion_mini_width,
                18,
            ),
            toggled=True,
        )
        self.motion_d_button = Button(
            "D",
            pygame.Rect(
                self.motion_c_button.rect.right + mini_gap,
                motion_mini_y,
                self.double_button.rect.width - motion_mini_width * 3 - mini_gap * 3,
                18,
            ),
            toggled=False,
        )
        mode_e_y = self.orange_button.rect.bottom + 6
        mode_ef_gap = 6
        mode_ef_width = (self.orange_button.rect.width - mode_ef_gap) // 2
        self.motion_e_button = Button(
            "E",
            pygame.Rect(self.orange_button.rect.left, mode_e_y, mode_ef_width, 18),
            toggled=False,
        )
        self.motion_f_button = Button(
            "F",
            pygame.Rect(
                self.motion_e_button.rect.right + mode_ef_gap,
                mode_e_y,
                self.orange_button.rect.width - mode_ef_width - mode_ef_gap,
                18,
            ),
            toggled=False,
        )
        self.reset_button.prepare_label(self.font_medium)
        self.double_button.prepare_label(self.font_medium)
        self.triple_button.prepare_label(self.font_medium)
        self.orange_button.prepare_label(self.font_medium)
        self.violett_button.prepare_label(self.font_medium)
        self.grid64_button.prepare_label(self.font_small)
        self.grid128_button.prepare_label(self.font_small)
        self.grid16_button.prepare_label(self.font_small)
        self.grid32_button.prepare_label(self.font_small)
        self.motion_a_button.prepare_label(self.font_small)
        self.motion_b_button.prepare_label(self.font_small)
        self.motion_c_button.prepare_label(self.font_small)
        self.motion_d_button.prepare_label(self.font_small)
        self.motion_e_button.prepare_label(self.font_small)
        self.motion_f_button.prepare_label(self.font_small)
        # The active composition is emitter-driven. Hard geometry helpers remain
        # in the solver but are not exposed in this UI.

    def allocate_grid_buffers(self, size: int) -> None:
        # Everything here depends on grid resolution. Rebuild buffers in one
        # place so switching between 16/32/64/128 always starts from consistent
        # solver and render state.
        self.solver = FluidSolver(size)
        self.surface = pygame.Surface((size, size))
        self.thumbnail_surface = pygame.Surface((size, size))
        # Reused render buffers keep the hot path array-based and avoid
        # repeated allocations during drawing.
        self.rgb_buffer = np.empty((size, size, 3), dtype=np.uint8)
        # Cache the main render before debug views overwrite rgb_buffer. The
        # thumbnails can reuse the same color-graded frame as their source.
        self.thumbnail_rgb_buffer = np.empty((size, size, 3), dtype=np.uint8)
        self.velocity_mag_buffer = np.empty((size, size), dtype=np.float32)
        self.debug_buffer = np.empty((size, size), dtype=np.float32)
        self.emitter_influence_buffer = np.zeros((size, size), dtype=np.float32)
        self.emitter_influence_r = np.zeros((size, size), dtype=np.float32)
        self.emitter_influence_g = np.zeros((size, size), dtype=np.float32)
        self.emitter_influence_b = np.zeros((size, size), dtype=np.float32)
        self.color_buffer_r = np.empty((size, size), dtype=np.float32)
        self.color_buffer_g = np.empty((size, size), dtype=np.float32)
        self.color_buffer_b = np.empty((size, size), dtype=np.float32)
        self.glow_buffer_r = np.empty((size, size), dtype=np.float32)
        self.glow_buffer_g = np.empty((size, size), dtype=np.float32)
        self.glow_buffer_b = np.empty((size, size), dtype=np.float32)
        self.emitter_influence_cache.clear()
        self.emitter_debug_arrows.clear()

    def set_grid_resolution(self, size: int) -> None:
        # Resolution switches are hard resets. Resampling velocity, pressure and
        # dye between different grid sizes is possible, but it adds complexity
        # and can introduce artifacts that are not useful for this sketch.
        global GRID_SIZE
        if size == GRID_SIZE:
            self.grid16_button.toggled = size == 16
            self.grid32_button.toggled = size == 32
            self.grid64_button.toggled = size == 64
            self.grid128_button.toggled = size == 128
            return
        GRID_SIZE = size
        self.allocate_grid_buffers(GRID_SIZE)
        self.grid16_button.toggled = size == 16
        self.grid32_button.toggled = size == 32
        self.grid64_button.toggled = size == 64
        self.grid128_button.toggled = size == 128

    def run(self) -> None:
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_r:
                        self.solver.clear()
                    elif event.key == pygame.K_TAB:
                        self.view_mode_index = (self.view_mode_index + 1) % len(self.view_modes)
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    if self.reset_button.hit_test(event.pos):
                        self.solver.clear()
                    elif self.grid16_button.hit_test(event.pos):
                        self.set_grid_resolution(16)
                    elif self.grid32_button.hit_test(event.pos):
                        self.set_grid_resolution(32)
                    elif self.grid64_button.hit_test(event.pos):
                        self.set_grid_resolution(64)
                    elif self.grid128_button.hit_test(event.pos):
                        self.set_grid_resolution(128)
                    elif self.motion_a_button.hit_test(event.pos):
                        self.set_emitter_motion_mode("A")
                    elif self.motion_b_button.hit_test(event.pos):
                        self.set_emitter_motion_mode("B")
                    elif self.motion_c_button.hit_test(event.pos):
                        self.set_emitter_motion_mode("C")
                    elif self.motion_d_button.hit_test(event.pos):
                        self.set_emitter_motion_mode("D")
                    elif self.motion_e_button.hit_test(event.pos):
                        self.set_emitter_motion_mode("E")
                    elif self.motion_f_button.hit_test(event.pos):
                        self.set_emitter_motion_mode("F")
                    elif self.double_button.hit_test(event.pos):
                        self.set_color_mode("double")
                    elif self.triple_button.hit_test(event.pos):
                        self.set_color_mode("triple")
                    elif self.orange_button.hit_test(event.pos):
                        self.set_color_mode("orange")
                    elif self.violett_button.hit_test(event.pos):
                        self.set_color_mode("violett")

                for slider in self.sliders.values():
                    slider.handle_event(event)

            self.update()
            self.draw()
            pygame.display.flip()
            self.clock.tick(0)

        pygame.quit()

    def update(self) -> None:
        if self.paused:
            return

        # Substeps are fixed at 1. This keeps timing predictable and makes the
        # slider values easier to interpret during interactive tuning.
        dt = self.sliders["dt"].value
        viscosity = self.sliders["viscosity"].value
        diffusion = self.sliders["diffusion"].value
        iterations = int(self.sliders["iterations"].value)
        vorticity = self.sliders["vorticity"].value
        swirl_amount = self.sliders["swirl_amount"].value
        swirl_scale = self.sliders["swirl_scale"].value
        swirl_speed = self.sliders["swirl_speed"].value
        chroma_drift = self.sliders["chroma_drift"].value
        velocity_dissipation = self.sliders["velocity_dissipation"].value
        dye_dissipation = self.sliders["dye_dissipation"].value
        color_contrast = self.sliders["color_contrast"].value
        black_level = self.sliders["black_level"].value
        masked_render_boost = self.sliders["masked_render_boost"].value
        emitter_density = self.sliders["emitter_density"].value
        emitter_upward = self.sliders["emitter_upward"].value
        emitter_radius = self.sliders["emitter_radius"].value
        jet_noise_speed = self.sliders["jet_noise_speed"].value
        jet_orbit_speed = self.sliders["jet_orbit_speed"].value
        emitter_hue_speed = self.sliders["emitter_hue_speed"].value
        self.sim_time += dt
        self.jet_orbit_phase_degrees = (self.jet_orbit_phase_degrees + jet_orbit_speed * dt * 360.0) % 360.0
        base_hue = (self.sim_time * emitter_hue_speed) % 1.0
        # Palette logic is kept explicit so each mode has a predictable visual
        # identity: hue-wheel relationships, a warm orange family, or a
        # red/blue/violet stage-lighting palette.
        if self.color_mode == "orange":
            if self.emitter_motion_mode == "C":
                # In Mode C, orange should read as a coherent warm family.
                # Slight hue/saturation/value offsets prevent all four corner
                # emitters from collapsing into perfectly identical sources.
                orange_variants = (
                    (36.5, 0.99, 0.98),
                    (39.2, 1.00, 0.97),
                    (38.0, 1.00, 1.00),
                    (37.1, 0.98, 0.99),
                    (38.0, 1.00, 1.00),
                )
                emitter_colors = [
                    colorsys.hsv_to_rgb(hue / 360.0, saturation, value)
                    for hue, saturation, value in orange_variants
                ]
            else:
                emitter_colors = [
                    colorsys.hsv_to_rgb(hue / 360.0, 1.0, 1.0)
                    for hue in (0.0, 22.0, 38.0, 54.0, 14.0)
                ]
        elif self.color_mode == "violett":
            emitter_colors = [
                colorsys.hsv_to_rgb(0.0 / 360.0, 1.0, 1.0),
                colorsys.hsv_to_rgb(230.0 / 360.0, 1.0, 1.0),
                colorsys.hsv_to_rgb(240.0 / 360.0, 1.0, 0.45),
                colorsys.hsv_to_rgb(214.0 / 360.0, 1.0, 0.85),
                colorsys.hsv_to_rgb(330.0 / 360.0, 1.0, 0.95),
            ]
        else:
            if self.color_mode == "triple":
                hue_offsets = (0.0, 120.0, 240.0, 60.0, 180.0)
            else:
                hue_offsets = (0.0, 170.0, 190.0, 85.0, 275.0)
            emitter_colors = [
                colorsys.hsv_to_rgb((base_hue + offset / 360.0) % 1.0, 1.0, 1.0)
                for offset in hue_offsets
            ]
        self.jet_noise_time += dt * jet_noise_speed
        noise_time = self.jet_noise_time_offset + self.jet_noise_time
        # Position and direction are intentionally independent. The emitter
        # layout defines where dye/force enters, while value noise modulates the
        # firing direction. Keeping these layers separate makes motion easier to
        # reason about and tune.
        noise_angles = {
            name: self.noise_angle_degrees(noise_time, *seed)
            for name, seed in self.jet_noise_seeds.items()
        }
        self.color_contrast = color_contrast
        self.black_level = black_level
        self.masked_render_boost = masked_render_boost
        self.emitter_density_for_debug = emitter_density
        self.should_render_emitter_influence = self.view_modes[self.view_mode_index] == "emitter"
        self.hard_source_bounds = None
        self.hard_source_color = (1.0, 0.0, 0.0)
        self.hard_source_overlays.clear()
        if self.should_render_emitter_influence:
            self.emitter_influence_buffer.fill(0.0)
            self.emitter_influence_r.fill(0.0)
            self.emitter_influence_g.fill(0.0)
            self.emitter_influence_b.fill(0.0)
            self.emitter_debug_arrows.clear()
        center = GRID_SIZE * 0.5
        emitter_ring_radius = GRID_SIZE * 0.25
        if self.emitter_motion_mode == "E":
            shared_angle = self.noise_angle_degrees(noise_time, *self.jet_noise_seeds["center"])
            self.emit_center_square_source(
                dt=dt,
                color=colorsys.hsv_to_rgb(base_hue, 1.0, 1.0),
                density=emitter_density,
                speed=emitter_upward,
                radius=emitter_radius,
                angle_degrees=shared_angle,
            )
        elif self.emitter_motion_mode == "F":
            shared_angle = self.noise_angle_degrees(noise_time, *self.jet_noise_seeds["center"])
            square_colors = [
                colorsys.hsv_to_rgb((base_hue + index * 20.0 / 360.0) % 1.0, 1.0, 1.0)
                for index in range(4)
            ]
            self.emit_center_square_row_sources(
                dt=dt,
                colors=square_colors,
                density=emitter_density,
                speed=emitter_upward,
                radius=emitter_radius,
                angle_degrees=shared_angle,
            )
        elif self.emitter_motion_mode == "D":
            shared_angle = self.noise_angle_degrees(noise_time, *self.jet_noise_seeds["center"])
            self.emit_center_pair_sources(
                dt=dt,
                colors=emitter_colors,
                density=emitter_density,
                speed=emitter_upward,
                radius=emitter_radius,
                angle_degrees=shared_angle,
                center=center,
            )
        elif self.emitter_motion_mode == "C":
            corner_offsets = [
                (self.value_noise(noise_time, *self.jet_noise_seeds[f"ring_{index}"]) * 2.0 - 1.0) * 30.0
                for index in range(4)
            ]
            self.emit_corner_sources(
                dt=dt,
                colors=emitter_colors,
                density=emitter_density,
                speed=emitter_upward,
                radius=emitter_radius,
                angle_offsets=corner_offsets,
                center=center,
            )
        elif self.emitter_motion_mode == "B":
            lissajous_colors = [
                colorsys.hsv_to_rgb((base_hue + offset / 360.0) % 1.0, 1.0, 1.0)
                for offset in (0.0, 70.0, 140.0)
            ]
            self.emit_lissajous_sources(
                dt=dt,
                colors=lissajous_colors,
                density=emitter_density,
                speed=emitter_upward,
                radius=emitter_radius,
                phase_degrees=self.jet_orbit_phase_degrees,
                phase_speed=jet_orbit_speed,
                center=center,
            )
        else:
            # Mode A: two opposite anchors on the virtual circle. This gives a
            # symmetric baseline mode with minimal source complexity.
            orbit_degrees = self.jet_orbit_phase_degrees
            for index, base_degrees in enumerate((90.0, 270.0)):
                ring_angle = math.radians(base_degrees + orbit_degrees)
                row = center + math.sin(ring_angle) * emitter_ring_radius
                col = center + math.cos(ring_angle) * emitter_ring_radius
                self.emit_stationary_source(
                    dt=dt,
                    color=emitter_colors[index],
                    density=emitter_density,
                    speed=emitter_upward,
                    radius=emitter_radius,
                    row=row,
                    col=col,
                    angle_offset_degrees=noise_angles[f"ring_{index}"],
                )
        self.solver.step(
            dt=dt,
            viscosity=viscosity,
            diffusion=diffusion,
            iterations=iterations,
            vorticity=vorticity,
            swirl_amount=swirl_amount,
            swirl_scale=swirl_scale,
            swirl_speed=swirl_speed,
            swirl_time=self.sim_time,
            chroma_drift=chroma_drift,
            velocity_dissipation=velocity_dissipation,
            dye_dissipation=dye_dissipation,
        )

    # UI state toggles. Simulation code receives mode names, not button objects,
    # so input handling remains separate from emitter logic.

    def set_color_mode(self, mode: str) -> None:
        self.color_mode = mode
        self.double_button.toggled = mode == "double"
        self.triple_button.toggled = mode == "triple"
        self.orange_button.toggled = mode == "orange"
        self.violett_button.toggled = mode == "violett"

    def set_emitter_motion_mode(self, mode: str) -> None:
        self.emitter_motion_mode = mode
        self.motion_a_button.toggled = mode == "A"
        self.motion_b_button.toggled = mode == "B"
        self.motion_c_button.toggled = mode == "C"
        self.motion_d_button.toggled = mode == "D"
        self.motion_e_button.toggled = mode == "E"
        self.motion_f_button.toggled = mode == "F"

    def _emit_hard_square(
        self,
        dt: float,
        color: tuple[float, float, float],
        density: float,
        speed: float,
        half_size: int,
        center_row: int,
        center_col: int,
        angle_degrees: float,
    ) -> None:
        row0 = max(1, center_row - half_size)
        row1 = min(GRID_SIZE, center_row + half_size - 1)
        col0 = max(1, center_col - half_size)
        col1 = min(GRID_SIZE, center_col + half_size - 1)
        if row0 > row1 or col0 > col1:
            return
        amount_scale = dt * 4.0
        base_row = -1.0
        base_col = 0.0
        angle_radians = math.radians(angle_degrees)
        direction = (
            base_row * math.cos(angle_radians) - base_col * math.sin(angle_radians),
            base_row * math.sin(angle_radians) + base_col * math.cos(angle_radians),
        )
        dye_amount = density * amount_scale
        velocity_amount = speed * amount_scale
        target = np.s_[row0 : row1 + 1, col0 : col1 + 1]
        # Hard-square sources are both visual anchors and fluid inputs. Redraw
        # the source area every frame so the emitter remains crisp while the
        # solver advects dye away from it.
        hard_amount = max(255.0, dye_amount)
        self.solver.dye_r[target] = hard_amount * color[0]
        self.solver.dye_g[target] = hard_amount * color[1]
        self.solver.dye_b[target] = hard_amount * color[2]
        self.solver.u[target] += direction[0] * velocity_amount
        self.solver.v[target] += direction[1] * velocity_amount
        self.hard_source_bounds = (row0, row1, col0, col1)
        self.hard_source_color = color
        self.hard_source_overlays.append((row0, row1, col0, col1, color))
        if self.should_render_emitter_influence:
            influence_target = np.s_[row0 - 1 : row1, col0 - 1 : col1]
            self.emitter_influence_buffer[influence_target] += 1.0
            self.emitter_influence_r[influence_target] += color[0]
            self.emitter_influence_g[influence_target] += color[1]
            self.emitter_influence_b[influence_target] += color[2]
            self.emitter_debug_arrows.append(
                ((row0 + row1) * 0.5, center_col, direction[0], direction[1], speed, color)
            )

    def emit_center_square_source(
        self,
        dt: float,
        color: tuple[float, float, float],
        density: float,
        speed: float,
        radius: float,
        angle_degrees: float,
    ) -> None:
        # Mode E: one hard square in the center. Unlike soft emitters, this uses
        # a rectangular source so the visible injection point remains graphic
        # and stable. Hue follows the global hue clock.
        half_size = max(1, int(round(radius * 0.5)))
        center = int(round(GRID_SIZE * 0.5))
        self._emit_hard_square(dt, color, density, speed, half_size, center, center, angle_degrees)

    def emit_center_square_row_sources(
        self,
        dt: float,
        colors: list[tuple[float, float, float]],
        density: float,
        speed: float,
        radius: float,
        angle_degrees: float,
    ) -> None:
        # Mode F: four adjacent hard squares. Their hues are spaced by 20
        # degrees and they share one velocity direction, producing a single
        # multi-color emitter bar rather than four independent jets.
        half_size = max(1, int(round(radius * 0.5)))
        side = max(1, half_size * 2)
        center_row = int(round(GRID_SIZE * 0.5))
        start_col = int(round(GRID_SIZE * 0.5 - side * 1.5))
        for index, color in enumerate(colors[:4]):
            center_col = start_col + index * side
            self._emit_hard_square(dt, color, density, speed, half_size, center_row, center_col, angle_degrees)

    def emit_center_pair_sources(
        self,
        dt: float,
        colors: list[tuple[float, float, float]],
        density: float,
        speed: float,
        radius: float,
        angle_degrees: float,
        center: float,
    ) -> None:
        # Mode D: two center emitters share one noise-driven direction. Their
        # centers are spaced by two radii so their splats remain visually
        # separate while still behaving as a paired source.
        direction = (math.sin(math.radians(angle_degrees)), math.cos(math.radians(angle_degrees)))
        half_distance = radius
        source_colors = colors[:2]
        positions = (
            (center, center - half_distance),
            (center, center + half_distance),
        )
        for color, (row, col) in zip(source_colors, positions):
            self.emit_stationary_source(
                dt=dt,
                color=color,
                density=density,
                speed=speed,
                radius=radius,
                row=row,
                col=col,
                angle_offset_degrees=0.0,
                base_direction=direction,
            )

    def emit_corner_sources(
        self,
        dt: float,
        colors: list[tuple[float, float, float]],
        density: float,
        speed: float,
        radius: float,
        angle_offsets: list[float],
        center: float,
    ) -> None:
        # Mode C: four corner emitters aim toward the center and apply a small
        # noise angle offset around that baseline. The radius-based inset keeps
        # the source splats inside the simulation domain.
        inset = clamp(radius * 0.75, 2.0, GRID_SIZE * 0.2)
        corner_positions = (
            (inset, inset),
            (inset, GRID_SIZE + 1.0 - inset),
            (GRID_SIZE + 1.0 - inset, GRID_SIZE + 1.0 - inset),
            (GRID_SIZE + 1.0 - inset, inset),
        )
        for index, (row, col) in enumerate(corner_positions):
            self.emit_stationary_source(
                dt=dt,
                color=colors[index],
                density=density,
                speed=speed,
                radius=radius,
                row=row,
                col=col,
                angle_offset_degrees=angle_offsets[index],
                base_direction=(center - row, center - col),
            )

    def emit_lissajous_sources(
        self,
        dt: float,
        colors: list[tuple[float, float, float]],
        density: float,
        speed: float,
        radius: float,
        phase_degrees: float,
        phase_speed: float,
        center: float,
    ) -> None:
        # Mode B: three emitters follow Lissajous paths. Their emission
        # direction is the negative tangent of the path, so the injected flow
        # trails their movement and produces calligraphic curved structures.
        phase = math.radians(phase_degrees)
        amplitude = GRID_SIZE * 0.31
        paths = (
            (1.0, 2.0, 0.0, 0.45, 1.00, 0.88),
            (2.0, 3.0, 1.1, 2.20, 0.82, 1.00),
            (3.0, 2.0, 2.4, 0.90, 1.00, 0.74),
        )
        phase_direction = -1.0 if phase_speed < 0.0 else 1.0
        for index, (freq_x, freq_y, phase_x, phase_y, scale_x, scale_y) in enumerate(paths):
            angle_x = phase * freq_x + phase_x
            angle_y = phase * freq_y + phase_y
            col = center + math.sin(angle_x) * amplitude * scale_x
            row = center + math.sin(angle_y) * amplitude * scale_y
            # Analytical derivatives provide the local path tangent. Only the
            # direction is needed; emission points against that tangent.
            d_col = math.cos(angle_x) * freq_x * amplitude * scale_x * phase_direction
            d_row = math.cos(angle_y) * freq_y * amplitude * scale_y * phase_direction
            self.emit_stationary_source(
                dt=dt,
                color=colors[index],
                density=density,
                speed=speed,
                radius=radius,
                row=row,
                col=col,
                angle_offset_degrees=0.0,
                base_direction=(-d_row, -d_col),
            )

    def value_noise(self, time_value: float, phase_a: float, phase_b: float) -> float:
        base = math.floor(time_value)
        frac = time_value - base
        blend = smoothstep(frac)
        value0 = 0.5 + 0.5 * math.sin(base * 12.9898 + phase_a) * math.cos(base * 78.233 + phase_b)
        value1 = 0.5 + 0.5 * math.sin((base + 1.0) * 12.9898 + phase_a) * math.cos((base + 1.0) * 78.233 + phase_b)
        return value0 + (value1 - value0) * blend

    def noise_angle_degrees(self, time_value: float, phase_a: float, phase_b: float) -> float:
        # Smooth value noise provides continuous angle changes without the hard
        # discontinuities of frame-to-frame random values.
        return self.value_noise(time_value, phase_a, phase_b) * 360.0

    def emit_stationary_source(
        self,
        dt: float,
        color: tuple[float, float, float],
        density: float,
        speed: float,
        radius: float,
        row: float,
        col: float,
        angle_offset_degrees: float,
        base_direction: tuple[float, float] | None = None,
    ) -> None:
        # Without an explicit direction, a source points away from the grid
        # center and then gets rotated by the angle offset. Center-based sources
        # pass their own direction because the outward vector is undefined at
        # the center.
        if base_direction is None:
            center_row = GRID_SIZE * 0.5
            center_col = GRID_SIZE * 0.5
            dir_row = row - center_row
            dir_col = col - center_col
            base_length = math.hypot(dir_row, dir_col)
            if base_length <= 1e-6:
                return
            dir_row /= base_length
            dir_col /= base_length
        else:
            dir_row, dir_col = base_direction
            base_length = math.hypot(dir_row, dir_col)
            if base_length <= 1e-6:
                return
            dir_row /= base_length
            dir_col /= base_length
        angle_radians = math.radians(angle_offset_degrees)
        velocity_x = dir_row * math.cos(angle_radians) - dir_col * math.sin(angle_radians)
        velocity_y = dir_row * math.sin(angle_radians) + dir_col * math.cos(angle_radians)
        amount_scale = dt * 4.0
        if self.should_render_emitter_influence:
            self.emitter_debug_arrows.append((row, col, velocity_x, velocity_y, speed, color))
        # Soft jets use three layered splats: a dense core, a broader middle
        # and a weaker tail. This gives the emitter a directional footprint
        # without needing a more expensive cone rasterizer.
        layers = (
            (0.55, 1.00, 0.00, 0.00),
            (0.30, 0.82, 0.00, -1.20),
            (0.15, 0.65, 0.00, -2.20),
        )

        for density_weight, velocity_weight, x_shift, y_shift in layers:
            sx = clamp(row + y_shift, 1.0, GRID_SIZE)
            sy = clamp(col + x_shift, 1.0, GRID_SIZE)
            sr = max(1.0, radius * (1.15 - 0.1 * abs(y_shift)))
            self.solver.add_dye_float(sx, sy, color, density * density_weight * amount_scale, sr)
            self.solver.add_velocity_float(
                sx,
                sy,
                velocity_x * speed * velocity_weight * amount_scale,
                velocity_y * speed * velocity_weight * amount_scale,
                sr,
            )
            if self.should_render_emitter_influence:
                self.add_emitter_influence(sx, sy, density_weight, sr, color)

    def add_emitter_influence(
        self,
        row: float,
        col: float,
        amount: float,
        radius: float,
        color: tuple[float, float, float],
    ) -> None:
        # Same radial idea as solver splats, used only for the emitter debug
        # map. Float centers make the preview show sub-cell motion accurately.
        # This is useful for debugging emitter placement, but it can be
        # quantized in a later performance-focused C++/FastLED port.
        r = max(1, int(math.ceil(radius)))
        row = clamp(row, 1.0, float(GRID_SIZE))
        col = clamp(col, 1.0, float(GRID_SIZE))
        row0 = max(1, int(math.floor(row - r)))
        row1 = min(GRID_SIZE, int(math.ceil(row + r)))
        col0 = max(1, int(math.floor(col - r)))
        col1 = min(GRID_SIZE, int(math.ceil(col + r)))
        if row0 > row1 or col0 > col1:
            return
        rows = np.arange(row0, row1 + 1, dtype=np.float32) - row
        cols = np.arange(col0, col1 + 1, dtype=np.float32) - col
        rr, cc = np.meshgrid(rows, cols, indexing="ij")
        weights = np.exp(
            -(rr * rr + cc * cc) / max(1e-5, radius * radius * 0.6),
            dtype=np.float32,
        )
        weighted = amount * weights
        target = np.s_[row0 - 1 : row1, col0 - 1 : col1]
        self.emitter_influence_buffer[target] += weighted
        self.emitter_influence_r[target] += weighted * color[0]
        self.emitter_influence_g[target] += weighted * color[1]
        self.emitter_influence_b[target] += weighted * color[2]

    def draw(self) -> None:
        self.screen.fill(BG_COLOR)

        # Start from dye plus a velocity magnitude estimate. The render path is
        # intentionally stylized: fast regions receive a small brightness lift
        # so motion remains legible on a low-resolution LED-style display.
        dye_r = self.solver.dye_r[1:-1, 1:-1]
        dye_g = self.solver.dye_g[1:-1, 1:-1]
        dye_b = self.solver.dye_b[1:-1, 1:-1]
        velocity_mag = self.velocity_mag_buffer
        np.square(self.solver.u[1:-1, 1:-1], out=velocity_mag)
        velocity_mag += self.solver.v[1:-1, 1:-1] * self.solver.v[1:-1, 1:-1]
        np.sqrt(velocity_mag, out=velocity_mag)
        velocity_mag *= 1.8
        np.clip(velocity_mag, 0.0, 255.0, out=velocity_mag)

        red = self.color_buffer_r
        green = self.color_buffer_g
        blue = self.color_buffer_b
        np.add(dye_r, velocity_mag * 0.08, out=red)
        np.add(dye_g, velocity_mag * 0.08, out=green)
        np.add(dye_b, velocity_mag * 0.08, out=blue)
        np.clip(red, 0.0, 255.0, out=red)
        np.clip(green, 0.0, 255.0, out=green)
        np.clip(blue, 0.0, 255.0, out=blue)
        velocity_mag *= 1.0 / 255.0
        red *= 1.0 / 255.0
        green *= 1.0 / 255.0
        blue *= 1.0 / 255.0

        # Highlights receive a saturation lift based on local luminance. This
        # counteracts the pastel look that can happen when several dye colors
        # diffuse together.
        np.maximum(red, green, out=velocity_mag)
        np.maximum(velocity_mag, blue, out=velocity_mag)
        velocity_mag -= 0.42
        np.clip(velocity_mag, 0.0, 1.0, out=velocity_mag)
        velocity_mag *= 1.0 / 0.58
        highlight_saturation = 1.0 + velocity_mag * self.highlight_saturation
        luminance = red * 0.2126 + green * 0.7152 + blue * 0.0722
        red[:] = luminance + (red - luminance) * highlight_saturation
        green[:] = luminance + (green - luminance) * highlight_saturation
        blue[:] = luminance + (blue - luminance) * highlight_saturation

        # A small blur is applied only to bright residuals, then added back as a
        # glow term. Limiting the blur to highlights preserves dark background
        # contrast and avoids washing out the whole frame.
        glow_r = self.glow_buffer_r
        glow_g = self.glow_buffer_g
        glow_b = self.glow_buffer_b
        np.maximum(red - 0.55, 0.0, out=glow_r)
        np.maximum(green - 0.55, 0.0, out=glow_g)
        np.maximum(blue - 0.55, 0.0, out=glow_b)
        glow_r[1:-1, 1:-1] = (
            glow_r[1:-1, 1:-1] * 0.42
            + glow_r[0:-2, 1:-1] * 0.145
            + glow_r[2:, 1:-1] * 0.145
            + glow_r[1:-1, 0:-2] * 0.145
            + glow_r[1:-1, 2:] * 0.145
        )
        glow_g[1:-1, 1:-1] = (
            glow_g[1:-1, 1:-1] * 0.42
            + glow_g[0:-2, 1:-1] * 0.145
            + glow_g[2:, 1:-1] * 0.145
            + glow_g[1:-1, 0:-2] * 0.145
            + glow_g[1:-1, 2:] * 0.145
        )
        glow_b[1:-1, 1:-1] = (
            glow_b[1:-1, 1:-1] * 0.42
            + glow_b[0:-2, 1:-1] * 0.145
            + glow_b[2:, 1:-1] * 0.145
            + glow_b[1:-1, 0:-2] * 0.145
            + glow_b[1:-1, 2:] * 0.145
        )
        red += glow_r * self.glow_strength
        green += glow_g * self.glow_strength
        blue += glow_b * self.glow_strength

        # Black point remaps low values toward zero before gamma/contrast. This
        # increases perceived contrast and keeps dim accumulated dye from making
        # the background look flat.
        red -= self.black_level
        green -= self.black_level
        blue -= self.black_level
        red *= 1.0 / (1.0 - self.black_level)
        green *= 1.0 / (1.0 - self.black_level)
        blue *= 1.0 / (1.0 - self.black_level)
        np.clip(red, 0.0, 1.0, out=red)
        np.clip(green, 0.0, 1.0, out=green)
        np.clip(blue, 0.0, 1.0, out=blue)
        gamma = 1.0 / max(0.2, self.color_contrast)

        self.rgb_buffer[:, :, 0] = (np.power(red, gamma) * 255.0).astype(np.uint8)
        self.rgb_buffer[:, :, 1] = (np.power(green, gamma) * 255.0).astype(np.uint8)
        self.rgb_buffer[:, :, 2] = (np.power(blue, gamma) * 255.0).astype(np.uint8)
        self.draw_hard_source_overlay()
        np.copyto(self.thumbnail_rgb_buffer, self.rgb_buffer)
        self.render_view_mode(self.view_modes[self.view_mode_index])
        self.draw_hard_source_overlay()
        pygame.surfarray.blit_array(self.surface, np.transpose(self.rgb_buffer, (1, 0, 2)))
        self.screen.fill((0, 0, 0))
        scaled = pygame.transform.scale(self.surface, self.sim_rect.size)
        self.screen.blit(scaled, self.sim_rect)
        pygame.draw.rect(self.screen, GRID_BORDER, self.sim_rect, width=2, border_radius=4)

        pygame.draw.rect(self.screen, PANEL_COLOR, self.panel_rect, border_radius=14)
        self.reset_button.draw(self.screen, self.font_medium)
        self.double_button.draw(self.screen, self.font_medium)
        self.triple_button.draw(self.screen, self.font_medium)
        self.orange_button.draw(self.screen, self.font_medium)
        self.violett_button.draw(self.screen, self.font_medium)
        self.grid16_button.draw(self.screen, self.font_small)
        self.grid32_button.draw(self.screen, self.font_small)
        self.grid64_button.draw(self.screen, self.font_small)
        self.grid128_button.draw(self.screen, self.font_small)
        self.motion_a_button.draw(self.screen, self.font_small)
        self.motion_b_button.draw(self.screen, self.font_small)
        self.motion_c_button.draw(self.screen, self.font_small)
        self.motion_d_button.draw(self.screen, self.font_small)
        self.motion_e_button.draw(self.screen, self.font_small)
        self.motion_f_button.draw(self.screen, self.font_small)

        for key in self.slider_order:
            self.sliders[key].draw(self.screen, self.font_medium, self.font_small)

        fps_value = self.clock.get_fps()
        throughput_mpx = (GRID_SIZE * GRID_SIZE * fps_value) / 1_000_000.0
        frame_ms = 1000.0 / fps_value if fps_value > 1e-6 else 0.0
        total_dye = float(
            np.sum(self.solver.dye_r[1:-1, 1:-1])
            + np.sum(self.solver.dye_g[1:-1, 1:-1])
            + np.sum(self.solver.dye_b[1:-1, 1:-1])
        )
        max_white_dye = GRID_SIZE * GRID_SIZE * 255.0 * 3.0
        # 100% means every grid cell is full white in all channels. Internal dye
        # may exceed that reference level, so the displayed metric is clamped.
        dye_percent = clamp(total_dye / max_white_dye * 100.0, 0.0, 999.9)
        view_label = self.view_labels[self.view_modes[self.view_mode_index]]
        view_text = self.font_medium.render(view_label, True, TEXT_COLOR)
        view_rect = view_text.get_rect(topright=(self.sim_rect.right, self.sim_rect.bottom + 12))
        status_x = self.sim_rect.left
        status_y = self.sim_rect.bottom + 12
        status_gap = 14
        status_x = self.draw_status_metric("FPS", f"{fps_value:3.0f}", status_x, status_y) + status_gap
        status_x = self.draw_status_metric("Frame", f"{frame_ms:4.1f}ms", status_x, status_y) + status_gap
        status_x = self.draw_status_metric("Throughput", f"{throughput_mpx:4.2f}MPx/s", status_x, status_y) + status_gap
        self.draw_status_metric("Dye", f"{dye_percent:3.0f}%", status_x, status_y)
        self.screen.blit(view_text, view_rect)

    def draw_hard_source_overlay(self) -> None:
        # Hard-square modes are source objects, not just advected dye. Draw them
        # directly into the beauty pass so their graphic source shape remains
        # crisp even while emitted dye is advected away.
        if self.emitter_motion_mode not in ("E", "F") or not self.hard_source_overlays:
            return
        if self.view_modes[self.view_mode_index] not in ("color", "masked_render"):
            return
        for row0, row1, col0, col1, color in self.hard_source_overlays:
            target = np.s_[row0 - 1 : row1, col0 - 1 : col1]
            self.rgb_buffer[target + (0,)] = int(255 * color[0])
            self.rgb_buffer[target + (1,)] = int(255 * color[1])
            self.rgb_buffer[target + (2,)] = int(255 * color[2])

    def draw_status_metric(self, label: str, value: str, x: int, y: int) -> int:
        # Lay metrics out by measured text width. This avoids overlap when
        # values change width at runtime.
        label_surface = self.font_status_label.render(f"{label}:", True, SUBTLE_TEXT)
        value_surface = self.font_status_value.render(value, True, TEXT_COLOR)
        self.screen.blit(label_surface, (x, y + 4))
        value_x = x + label_surface.get_width() + 5
        self.screen.blit(value_surface, (value_x, y))
        return value_x + value_surface.get_width()

    def draw_debug_thumbnails(self) -> None:
        # Thumbnails are live previews of the TAB modes. They are intentionally
        # small so they remain cheap while still exposing the current state of
        # each diagnostic buffer.
        active_mode = self.view_modes[self.view_mode_index]
        for mode in self.view_modes:
            rect = self.thumbnail_rects[mode]
            self.render_view_mode(mode)
            pygame.surfarray.blit_array(self.thumbnail_surface, np.transpose(self.rgb_buffer, (1, 0, 2)))
            scaled = pygame.transform.scale(self.thumbnail_surface, rect.size)
            self.screen.blit(scaled, rect)
            border_color = TEXT_COLOR if mode == active_mode else GRID_BORDER
            pygame.draw.rect(self.screen, border_color, rect, width=2, border_radius=4)

    def render_view_mode(self, mode: str) -> None:
        # Every view writes into rgb_buffer. This keeps the draw path uniform:
        # render a buffer, upload it to a pygame surface, then scale/blit.
        if mode == "color":
            np.copyto(self.rgb_buffer, self.thumbnail_rgb_buffer)
        elif mode == "masked_render":
            self.render_masked_render_view()
        elif mode == "velocity":
            self.render_velocity_view()
        elif mode == "vorticity":
            self.render_vorticity_view()
        elif mode == "pressure":
            self.render_pressure_view()
        elif mode == "divergence":
            self.render_divergence_view()
        elif mode == "emitter":
            self.render_emitter_influence_view()
        elif mode == "dye_density":
            self.render_dye_density_view()

    def render_masked_render_view(self) -> None:
        # Masked Render starts from the main color render, applies a controlled
        # brightness/saturation boost, then multiplies by the same fixed-scale
        # velocity field used in Velocity Map. Slow regions darken, fast regions
        # remain bright. True black remains black because it is multiplied by
        # the mask after the boost.
        velocity_mag = self.write_static_velocity_field()
        velocity_mag *= 1.0 / 255.0
        red = self.color_buffer_r
        green = self.color_buffer_g
        blue = self.color_buffer_b
        red[:, :] = self.thumbnail_rgb_buffer[:, :, 0] * (1.0 / 255.0)
        green[:, :] = self.thumbnail_rgb_buffer[:, :, 1] * (1.0 / 255.0)
        blue[:, :] = self.thumbnail_rgb_buffer[:, :, 2] * (1.0 / 255.0)
        luminance = red * 0.2126 + green * 0.7152 + blue * 0.0722
        boost = self.masked_render_boost
        saturation_boost = 1.0 + boost * 0.42
        brightness_boost = 1.0 + boost * 0.28
        red[:] = (luminance + (red - luminance) * saturation_boost) * brightness_boost
        green[:] = (luminance + (green - luminance) * saturation_boost) * brightness_boost
        blue[:] = (luminance + (blue - luminance) * saturation_boost) * brightness_boost
        np.clip(red, 0.0, 1.0, out=red)
        np.clip(green, 0.0, 1.0, out=green)
        np.clip(blue, 0.0, 1.0, out=blue)
        self.rgb_buffer[:, :, 0] = (red * velocity_mag * 255.0).astype(np.uint8)
        self.rgb_buffer[:, :, 1] = (green * velocity_mag * 255.0).astype(np.uint8)
        self.rgb_buffer[:, :, 2] = (blue * velocity_mag * 255.0).astype(np.uint8)

    def render_velocity_view(self) -> None:
        # Velocity view: speed magnitude as a fixed-scale red heat map. It is
        # not normalized per frame, so brightness differences correspond to
        # absolute velocity differences under the chosen scale.
        velocity_mag = self.write_static_velocity_field()
        self.rgb_buffer[:, :, 0] = velocity_mag.astype(np.uint8)
        self.rgb_buffer[:, :, 1] = 0
        self.rgb_buffer[:, :, 2] = 0

    def write_static_velocity_field(self) -> np.ndarray:
        velocity_mag = self.velocity_mag_buffer
        np.square(self.solver.u[1:-1, 1:-1], out=velocity_mag)
        velocity_mag += self.solver.v[1:-1, 1:-1] * self.solver.v[1:-1, 1:-1]
        np.sqrt(velocity_mag, out=velocity_mag)
        velocity_mag *= 110.0
        np.clip(velocity_mag, 0.0, 255.0, out=velocity_mag)
        return velocity_mag

    def render_vorticity_view(self) -> None:
        # Vorticity view: absolute curl magnitude in blue. This shows where the
        # velocity field contains local rotational motion.
        curl = self.debug_buffer
        curl[:, :] = np.abs(
            0.5
            * (
                self.solver.v[2:, 1:-1]
                - self.solver.v[0:-2, 1:-1]
                - self.solver.u[1:-1, 2:]
                + self.solver.u[1:-1, 0:-2]
            )
        )
        self.write_single_channel_view(curl, red=0.0, green=0.0, blue=1.0, eps=1e-8)

    def render_pressure_view(self) -> None:
        # Pressure view: absolute value of the most recent pressure solve. This
        # indicates where projection applied the strongest scalar correction.
        pressure = self.debug_buffer
        pressure[:, :] = np.abs(self.solver.pressure[1:-1, 1:-1])
        self.write_single_channel_view(pressure, red=0.0, green=1.0, blue=0.0, eps=1e-8)

    def render_divergence_view(self) -> None:
        # Divergence view keeps expansion and compression separate. Positive
        # divergence maps toward yellow, negative divergence toward green. Large
        # values indicate areas where the velocity field is less incompressible.
        divergence = self.debug_buffer
        divergence[:, :] = 0.5 * (
            self.solver.u[2:, 1:-1]
            - self.solver.u[0:-2, 1:-1]
            + self.solver.v[1:-1, 2:]
            - self.solver.v[1:-1, 0:-2]
        )
        max_abs = float(np.max(np.abs(divergence)))
        if max_abs > 1e-8:
            divergence *= 255.0 / max_abs
        else:
            divergence.fill(0.0)
        positive = np.clip(divergence, 0.0, 255.0)
        negative = np.clip(-divergence, 0.0, 255.0)
        self.rgb_buffer[:, :, 0] = positive.astype(np.uint8)
        self.rgb_buffer[:, :, 1] = np.maximum(positive, negative).astype(np.uint8)
        self.rgb_buffer[:, :, 2] = 0

    def render_emitter_influence_view(self) -> None:
        # Emitter view: where sources inject dye and velocity this frame,
        # colored with the active palette. This helps verify source placement,
        # color relationships and emission direction independently from the
        # accumulated fluid render.
        density_scale = max(0.0, self.emitter_density_for_debug / 120.0)
        # Static density-based scaling avoids per-frame auto-exposure. A higher
        # emitter density produces a brighter debug view, matching the actual
        # source strength instead of normalizing it away.
        scale = 255.0 * density_scale / 1.45
        if self.emitter_density_for_debug > 1e-8:
            np.multiply(self.emitter_influence_r, scale, out=self.color_buffer_r)
            np.multiply(self.emitter_influence_g, scale, out=self.color_buffer_g)
            np.multiply(self.emitter_influence_b, scale, out=self.color_buffer_b)
            np.clip(self.color_buffer_r, 0.0, 255.0, out=self.color_buffer_r)
            np.clip(self.color_buffer_g, 0.0, 255.0, out=self.color_buffer_g)
            np.clip(self.color_buffer_b, 0.0, 255.0, out=self.color_buffer_b)
            self.rgb_buffer[:, :, 0] = self.color_buffer_r.astype(np.uint8)
            self.rgb_buffer[:, :, 1] = self.color_buffer_g.astype(np.uint8)
            self.rgb_buffer[:, :, 2] = self.color_buffer_b.astype(np.uint8)
        else:
            self.rgb_buffer.fill(0)
        self.draw_emitter_arrows()

    def draw_emitter_arrows(self) -> None:
        # Influence blobs show where source energy enters the grid. White
        # anti-aliased arrows add direction and relative force, which are not
        # visible from the scalar influence field alone.
        for row, col, dir_row, dir_col, force, _color in self.emitter_debug_arrows:
            length = 3.0 + clamp(force, 0.0, 3.0) * 5.0
            start_row = row - 1.0
            start_col = col - 1.0
            end_row = start_row + dir_row * length
            end_col = start_col + dir_col * length
            rgb = (255, 255, 255)
            self.draw_buffer_line(start_row, start_col, end_row, end_col, rgb)

            head_length = 2.4
            head_angle = 0.62
            base_angle = math.atan2(dir_row, dir_col)
            for sign in (-1.0, 1.0):
                wing_angle = base_angle + math.pi + sign * head_angle
                wing_row = end_row + math.sin(wing_angle) * head_length
                wing_col = end_col + math.cos(wing_angle) * head_length
                self.draw_buffer_line(end_row, end_col, wing_row, wing_col, rgb)

    def draw_buffer_line(
        self,
        row0: float,
        col0: float,
        row1: float,
        col1: float,
        color: tuple[int, int, int],
    ) -> None:
        steps = max(1, int(math.ceil(max(abs(row1 - row0), abs(col1 - col0)) * 3.0)))
        for index in range(steps + 1):
            t = index / steps
            row = row0 + (row1 - row0) * t
            col = col0 + (col1 - col0) * t
            self.draw_antialiased_point(row, col, color)

    def draw_antialiased_point(self, row: float, col: float, color: tuple[int, int, int]) -> None:
        row_base = math.floor(row)
        col_base = math.floor(col)
        for rr in range(row_base - 1, row_base + 2):
            if rr < 0 or rr >= GRID_SIZE:
                continue
            row_weight = max(0.0, 1.0 - abs(row - rr))
            for cc in range(col_base - 1, col_base + 2):
                if cc < 0 or cc >= GRID_SIZE:
                    continue
                weight = row_weight * max(0.0, 1.0 - abs(col - cc))
                if weight <= 0.0:
                    continue
                current = self.rgb_buffer[rr, cc].astype(np.float32)
                target = np.array(color, dtype=np.float32)
                self.rgb_buffer[rr, cc] = np.maximum(current, target * weight).astype(np.uint8)

    def render_dye_density_view(self) -> None:
        # Dye density view: total scalar dye amount independent of hue, shown in
        # yellow. This separates material density from palette choice.
        density = self.debug_buffer
        density[:, :] = (
            self.solver.dye_r[1:-1, 1:-1]
            + self.solver.dye_g[1:-1, 1:-1]
            + self.solver.dye_b[1:-1, 1:-1]
        )
        self.write_single_channel_view(density, red=1.0, green=0.86, blue=0.0, eps=1e-6)

    def write_single_channel_view(
        self,
        field: np.ndarray,
        red: float,
        green: float,
        blue: float,
        eps: float,
    ) -> None:
        max_value = float(np.max(field))
        if max_value > eps:
            field *= 255.0 / max_value
        else:
            field.fill(0.0)
        np.clip(field, 0.0, 255.0, out=field)
        self.rgb_buffer[:, :, 0] = (field * red).astype(np.uint8)
        self.rgb_buffer[:, :, 1] = (field * green).astype(np.uint8)
        self.rgb_buffer[:, :, 2] = (field * blue).astype(np.uint8)

    def write_grayscale_view(self, field: np.ndarray, eps: float) -> None:
        max_value = float(np.max(field))
        if max_value > eps:
            field *= 255.0 / max_value
        else:
            field.fill(0.0)
        np.clip(field, 0.0, 255.0, out=field)
        gray = field.astype(np.uint8)
        self.rgb_buffer[:, :, 0] = gray
        self.rgb_buffer[:, :, 1] = gray
        self.rgb_buffer[:, :, 2] = gray


def main() -> None:
    FluidApp().run()


if __name__ == "__main__":
    main()