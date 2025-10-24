"""
Optimized 3D Airplane Game with Endless Space
- Smooth airplane controls with acceleration based on joystick input
- High-performance endless space environment
- Optimized rendering and physics
- Responsive controls with deadzone handling
"""

from ursina import *
from ursina.shaders import lit_with_shadows_shader
import random
import math
from dataclasses import dataclass
from typing import List
import numpy as np

# ----------------- CONFIGURATION -----------------
WINDOW_TITLE = "3D Airplane in Space"
FULLSCREEN = False
VSYNC = True  # Enable vertical sync for smoother rendering
TARGET_FPS = 144  # Target frames per second

# Gameplay settings
PLANE_SPEED_MIN = 5.0
PLANE_SPEED_MAX = 200.0
PLANE_ACCELERATION = 0.5
PLANE_TURN_RATE = 80.0  # degrees per second
PLANE_ROLL_RATE = 100.0
PLANE_PITCH_RATE = 60.0
PLANE_DRAG = 0.99  # Air resistance (0.99 = 1% speed loss per second)

# Visual settings
STAR_COUNT = 2000  # Increased number of stars for better space feel
STAR_FIELD_RADIUS = 500
STAR_SIZE_RANGE = (0.05, 0.3)
FAR_CLIP = 1000  # Far clipping plane

# ----------------- GAME OBJECTS -----------------
@dataclass
class Star:
    position: Vec3
    size: float
    entity: Entity = None

class OptimizedAirplane(Entity):
    def __init__(self, **kwargs):
        super().__init__(
            model='cube',
            color=color.white,
            scale=(2, 0.6, 4),
            collider='box',
            shader=lit_with_shadows_shader,
            **kwargs
        )
        
        # Physics properties
        self.velocity = Vec3(0, 0, 0)
        self.acceleration = Vec3(0, 0, 0)
        self.speed = PLANE_SPEED_MIN
        self.target_speed = PLANE_SPEED_MIN
        
        # Control surfaces
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        
        # Control inputs (normalized -1 to 1)
        self.input_roll = 0
        self.input_pitch = 0
        self.input_yaw = 0
        self.input_throttle = 0  # 0 to 1
        
        # Visual effects
        self.trail = []
        self.last_trail_time = 0
        self.trail_interval = 0.02  # seconds between trail particles
        
        # Performance optimization
        self._last_update_time = 0
        
    def update(self):
        current_time = time.time()
        dt = min(time.dt, 1/30)  # Cap delta time to prevent physics issues
        
        # Update control surfaces based on input
        self.update_controls(dt)
        
        # Apply physics
        self.apply_physics(dt)
        
        # Update position
        self.position += self.velocity * dt
        
        # Update rotation
        self.rotation = Vec3(
            self.pitch,
            self.yaw,
            self.roll
        )
        
        # Update trail effect
        self.update_trail(current_time)
        
        self._last_update_time = current_time
    
    def update_controls(self, dt):
        """Update control surfaces based on input."""
        # Smooth input interpolation
        smooth_factor = min(1.0, dt * 5.0)
        
        # Apply input to control surfaces with smoothing
        self.roll = lerp(self.roll, self.input_roll * 45, smooth_factor)
        self.pitch = lerp(self.pitch, self.input_pitch * 30, smooth_factor)
        
        # Yaw is affected by roll (coordinated turn)
        yaw_input = self.input_yaw * 0.7 + (self.roll / 45.0) * 0.3
        self.yaw += yaw_input * PLANE_TURN_RATE * dt
        
        # Speed control
        self.target_speed = lerp(PLANE_SPEED_MIN, PLANE_SPEED_MAX, (self.input_throttle + 1) / 2)
        self.speed = lerp(self.speed, self.target_speed, PLANE_ACCELERATION * dt)
    
    def apply_physics(self, dt):
        """Apply physics to the airplane."""
        # Calculate forward vector from rotation
        forward = self.forward
        
        # Apply thrust in the forward direction
        thrust = forward * self.speed
        
        # Apply lift (simplified)
        lift = Vec3(0, math.sin(self.pitch * 0.0174533) * 0.2, 0)  # Convert degrees to radians
        
        # Update velocity
        self.velocity = (thrust + lift) * PLANE_DRAG
        
        # Apply gravity (simplified)
        self.velocity.y -= 9.8 * dt if self.position.y > -10 else 0
        
        # Keep the plane above ground
        if self.position.y < -10:
            self.position.y = -10
            self.velocity.y = max(0, self.velocity.y)
    
    def update_trail(self, current_time):
        """Update the visual trail effect."""
        if current_time - self.last_trail_time > self.trail_interval:
            # Create a new trail particle
            trail_pos = self.position - (self.forward * 2)  # Position behind the plane
            trail = Entity(
                model='sphere',
                position=trail_pos,
                scale=0.3,
                color=color.cyan,
                alpha=0.7,
                add_to_scene_entities=True
            )
            self.trail.append((current_time, trail))
            self.last_trail_time = current_time
        
        # Update and clean up old trail particles
        for i, (spawn_time, particle) in enumerate(self.trail[:]):
            # Fade out and scale down
            age = current_time - spawn_time
            if age > 1.0:  # Particle lifetime in seconds
                destroy(particle)
                self.trail.remove((spawn_time, particle))
            else:
                particle.alpha = 0.7 * (1.0 - age)
                particle.scale = 0.3 * (1.0 - age * 0.8)

class SpaceEnvironment:
    def __init__(self, star_count=STAR_COUNT):
        self.stars: List[Star] = []
        self.star_field_radius = STAR_FIELD_RADIUS
        self.player = None
        self.star_scale = 0.1
        
        # Create star field
        self.create_star_field(star_count)
    
    def create_star_field(self, count):
        """Create a field of stars in a spherical distribution."""
        # Use numpy for faster array operations
        theta = np.random.uniform(0, 2*np.pi, count)
        phi = np.arccos(2 * np.random.uniform(0, 1, count) - 1)
        r = self.star_field_radius * np.cbrt(np.random.uniform(0, 1, count))
        
        # Convert spherical to cartesian coordinates
        x = r * np.sin(phi) * np.cos(theta)
        y = r * np.sin(phi) * np.sin(theta)
        z = r * np.cos(phi)
        
        # Create star entities
        for i in range(count):
            pos = Vec3(x[i], y[i], z[i])
            size = random.uniform(*STAR_SIZE_RANGE)
            star = Star(
                position=pos,
                size=size,
                entity=Entity(
                    model='sphere',
                    position=pos,
                    scale=size,
                    color=color.white,
                    shader=lit_with_shadows_shader
                )
            )
            self.stars.append(star)
    
    def update(self, player_pos):
        """Update star positions to create an endless space effect."""
        for star in self.stars:
            # Calculate distance from player
            offset = star.position - player_pos
            dist = offset.length()
            
            # If star is too far away, move it to the opposite side
            if dist > self.star_field_radius * 0.9:
                # Move star to opposite side of the field
                star.position = player_pos - (offset.normalized() * self.star_field_radius * 0.8)
                star.entity.position = star.position

class AirplaneGame(Entity):
    def __init__(self):
        super().__init__()
        self.setup_window()
        self.setup_scene()
        self.setup_airplane()
        self.setup_controls()
        
        # Performance monitoring
        self.fps_counter = Text("FPS: 0", position=(-0.8, 0.45), scale=1.5)
        self.last_fps_update = 0
        self.fps_samples = []
    
    def setup_window(self):
        """Configure the game window."""
        window.title = WINDOW_TITLE
        window.borderless = False
        window.fullscreen = FULLSCREEN
        window.exit_button.visible = False
        window.vsync = VSYNC
        window.fps_counter.enabled = False  # We'll use our own FPS counter
        
        # Hide mouse cursor
        mouse.visible = False
    
    def setup_scene(self):
        """Set up the 3D scene."""
        # Create a black skybox
        sky = Entity(
            model='sphere',
            scale=1000,
            double_sided=True,
            color=color.black
        )
        
        # Lighting
        DirectionalLight(parent=scene, y=2, z=3, shadows=True)
        AmbientLight(parent=scene, color=color.gray * 0.2)
        
        # Create space environment
        self.space = SpaceEnvironment()
        
        # Camera setup
        self.camera_pivot = Entity()
        camera.parent = self.camera_pivot
        camera.position = (0, 1, -10)
        camera.rotation_x = 15
    
    def setup_airplane(self):
        """Create and configure the player's airplane."""
        self.airplane = OptimizedAirplane()
        self.airplane.position = (0, 10, 0)
        
        # Set camera to follow airplane
        self.camera_pivot.parent = self.airplane
        self.camera_pivot.position = (0, 1, 0)
    
    def setup_controls(self):
        """Set up keyboard controls."""
        # Mouse control
        self.mouse_sensitivity = 50
        window.show_ursina_splash = False
        mouse.visible = False
        window.exit_button.visible = False
        
        # Lock mouse to window for first-person controls
        if not window.fullscreen:
            window.position = (100, 100)
        
        # Hide the FPS counter
        window.fps_counter.enabled = False
    
    def input(self, key):
        """Handle keyboard input."""
        # Toggle fullscreen with F11
        if key == 'f11':
            window.fullscreen = not window.fullscreen
        
        # Exit with Escape
        if key == 'escape':
            application.quit()
    
    def update(self):
        """Main game loop."""
        # Update airplane controls based on input
        self.update_airplane_controls()
        
        # Update space environment
        self.space.update(self.airplane.position)
        
        # Update camera position (smooth follow)
        self.update_camera()
        
        # Update FPS counter
        self.update_fps_counter()
    
    def update_airplane_controls(self):
        """Update airplane controls based on keyboard input."""
        # Roll (A/D keys)
        self.airplane.input_roll = held_keys['d'] - held_keys['a']
        
        # Pitch (W/S keys)
        self.airplane.input_pitch = held_keys['s'] - held_keys['w']
        
        # Yaw (Q/E keys)
        self.airplane.input_yaw = held_keys['e'] - held_keys['q']
        
        # Throttle (Shift/Ctrl)
        self.airplane.input_throttle = held_keys['shift'] - held_keys['control']
    
    def update_camera(self):
        """Update camera position and rotation."""
        # Camera follows airplane with slight delay
        target_pos = (0, 2, -10)
        camera.position = lerp(camera.position, target_pos, time.dt * 5)
        
        # Look slightly ahead of the airplane
        look_target = Vec3(0, 0, 5)
        camera.look_at(look_target)
    
    def update_fps_counter(self):
        """Update the FPS counter with a rolling average."""
        current_time = time.time()
        self.fps_samples.append(1 / time.dt)
        
        if current_time - self.last_fps_update > 0.5:  # Update every 0.5 seconds
            avg_fps = sum(self.fps_samples) / len(self.fps_samples)
            self.fps_counter.text = f"FPS: {int(avg_fps)}"
            self.fps_samples = []
            self.last_fps_update = current_time

if __name__ == '__main__':
    # Start the game
    app = Ursina()
    game = AirplaneGame()
    app.run()
