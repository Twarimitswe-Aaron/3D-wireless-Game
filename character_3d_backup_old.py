"""
Enhanced 3D Car Racing Game with Realistic Physics
--------------------------------------------------
Requirements:
    pip install pygame PyOpenGL pyserial

Controls:
    - Arrow keys or Arduino Bluetooth joystick
    - Up: accelerate
    - Down: brake/reverse
    - Left/Right: steer
    - SPACE: handbrake
    - SHIFT: boost
    - C: change camera
    - R: reset position
"""

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import math
import serial
import serial.tools.list_ports
import threading
import queue
import time
import re

class RealisticCar:
    """Enhanced 3D car with realistic appearance and physics"""
    def __init__(self):
        self.position = [0, 0, -10]
        self.rotation = [0, 0, 0]  # [pitch, yaw, roll]
        self.velocity = [0, 0, 0]
        self.speed = 0
        self.max_speed = 2.0
        self.turn_speed = 3.0
        self.acceleration = 0.08
        self.braking = 0.15
        self.drift_factor = 0.95
        self.steering_angle = 0
        self.max_steering = 40
        self.wheel_rotation = 0
        self.boost = 1.0
        self.boost_amount = 2.0
        self.boost_recharge = 0.01
        self.boost_consumption = 0.05
        self.is_boosting = False
        
        # Car dimensions (more realistic proportions)
        self.car_length = 4.5
        self.car_width = 2.0
        self.car_height = 1.4
        self.wheelbase = 2.8  # Distance between front and rear axles
        self.track_width = 1.8  # Distance between left and right wheels
        
        # Wheel parameters
        self.wheel_radius = 0.35
        self.wheel_width = 0.25
        self.wheel_offset = 0.1  # How far wheels stick out from body
        
        # Colors - Sports car theme
        self.colors = {
            'body': (0.9, 0.1, 0.1),      # Red
            'body_dark': (0.6, 0.05, 0.05),  # Dark red
            'windows': (0.15, 0.2, 0.3),   # Dark blue-tinted
            'wheels': (0.05, 0.05, 0.05),  # Nearly black
            'rims': (0.7, 0.7, 0.75),      # Metallic gray
            'headlights': (1.0, 1.0, 0.9), # Bright white-yellow
            'taillights': (1.0, 0.1, 0.1), # Red
            'grille': (0.2, 0.2, 0.2),     # Dark gray
            'underbody': (0.15, 0.15, 0.15), # Very dark gray
            'brake_lights': (1.0, 0.0, 0.0)  # Bright red for braking
        }
        
        self.is_braking = False
        
        # Initialize display lists
        self.display_lists = {}
        self.init_display_lists()
    
    def init_display_lists(self):
        # Create display lists for static car parts
        self.display_lists['body'] = glGenLists(1)
        glNewList(self.display_lists['body'], GL_COMPILE)
        self._draw_car_body()
        glEndList()
        
        # Create display list for wheel (without rotation)
        self.display_lists['wheel'] = glGenLists(1)
        glNewList(self.display_lists['wheel'], GL_COMPILE)
        self._draw_wheel_geometry()
        glEndList()
    
    def _draw_car_body(self):
        """Draw the car body without pushing/popping matrices"""
        # Main body (lower section)
        glPushMatrix()
        glTranslatef(0, self.car_height * 0.25, 0)
        glScalef(self.car_width/2, self.car_height * 0.5/2, self.car_length/2)
        self._draw_cube()
        glPopMatrix()
        
        # Cabin (upper section)
        glPushMatrix()
        glTranslatef(0, self.car_height * 0.65, -0.3)
        glScalef(self.car_width * 0.85/2, self.car_height * 0.35/2, self.car_length * 0.3/2)
        self._draw_cube()
        glPopMatrix()
        
        # Windows and other details...
        # (Add the rest of the car body details here, but with minimal matrix operations)
    
    def _draw_wheel_geometry(self):
        """Draw a single wheel without position/rotation"""
        # Draw tire (rubber part)
        glColor3f(*self.colors['wheels'])
        quad = gluNewQuadric()
        gluQuadricNormals(quad, GLU_SMOOTH)
        gluCylinder(quad, self.wheel_radius, self.wheel_radius, self.wheel_width, 16, 1)
        
        # Draw rim (metal part) - slightly smaller
        glPushMatrix()
        glTranslatef(0, 0, self.wheel_width * 0.1)
        glColor3f(*self.colors['rims'])
        gluCylinder(quad, self.wheel_radius * 0.6, self.wheel_radius * 0.6, 
                   self.wheel_width * 0.8, 12, 1)
        glPopMatrix()
        
        # Draw spokes
        for i in range(5):
            glPushMatrix()
            glRotatef(i * 72, 0, 0, 1)
            glTranslatef(0, 0, self.wheel_width * 0.4)
            glScalef(0.08, self.wheel_radius * 0.5, 0.05)
            self._draw_cube()
            glPopMatrix()
        
        gluDeleteQuadric(quad)
    
    def _draw_cube(self):
        """Draw a unit cube centered at origin"""
        glBegin(GL_QUADS)
        # Front face
        glNormal3f(0, 0, 1)
        glVertex3f(-1, -1, 1); glVertex3f(1, -1, 1)
        glVertex3f(1, 1, 1); glVertex3f(-1, 1, 1)
        
        # Back face
        glNormal3f(0, 0, -1)
        glVertex3f(-1, -1, -1); glVertex3f(-1, 1, -1)
        glVertex3f(1, 1, -1); glVertex3f(1, -1, -1)
        
        # Top face
        glNormal3f(0, 1, 0)
        glVertex3f(-1, 1, -1); glVertex3f(-1, 1, 1)
        glVertex3f(1, 1, 1); glVertex3f(1, 1, -1)
        
        # Bottom face
        glNormal3f(0, -1, 0)
        glVertex3f(-1, -1, -1); glVertex3f(1, -1, -1)
        glVertex3f(1, -1, 1); glVertex3f(-1, -1, 1)
        
        # Left face
        glNormal3f(-1, 0, 0)
        glVertex3f(-1, -1, -1); glVertex3f(-1, -1, 1)
        glVertex3f(-1, 1, 1); glVertex3f(-1, 1, -1)
        
        # Right face
        glNormal3f(1, 0, 0)
        glVertex3f(1, -1, -1); glVertex3f(1, 1, -1)
        glVertex3f(1, 1, 1); glVertex3f(1, -1, 1)
        glEnd()

    def set_boost(self, boosting):
        """Set boost state"""
        self.is_boosting = boosting

    def draw_box(self, width, height, depth, color):
        """Draw a colored box with proper normals for lighting"""
        # Check matrix stack depth before pushing
        max_depth = glGetIntegerv(GL_MAX_MODELVIEW_STACK_DEPTH)
        current_depth = glGetInteger(GL_MODELVIEW_STACK_DEPTH)
        if current_depth >= max_depth - 5:  # Leave some room
            print(f"Warning: Matrix stack depth getting high: {current_depth}/{max_depth}")
            
        glPushMatrix()
        glScalef(width/2, height/2, depth/2)
        
        glBegin(GL_QUADS)
        glColor3f(*color)
        
        # Front face
        glNormal3f(0, 0, 1)
        glVertex3f(-1, -1, 1); glVertex3f(1, -1, 1)
        glVertex3f(1, 1, 1); glVertex3f(-1, 1, 1)
        
        # Back face
        glNormal3f(0, 0, -1)
        glVertex3f(-1, -1, -1); glVertex3f(-1, 1, -1)
        glVertex3f(1, 1, -1); glVertex3f(1, -1, -1)
        
        # Top face
        glNormal3f(0, 1, 0)
        glVertex3f(-1, 1, -1); glVertex3f(-1, 1, 1)
        glVertex3f(1, 1, 1); glVertex3f(1, 1, -1)
        
        # Bottom face
        glNormal3f(0, -1, 0)
        glVertex3f(-1, -1, -1); glVertex3f(1, -1, -1)
        glVertex3f(1, -1, 1); glVertex3f(-1, -1, 1)
        
        # Left face
        glNormal3f(-1, 0, 0)
        glVertex3f(-1, -1, -1); glVertex3f(-1, -1, 1)
        glVertex3f(-1, 1, 1); glVertex3f(-1, 1, -1)
        
        # Right face
        glNormal3f(1, 0, 0)
        glVertex3f(1, -1, -1); glVertex3f(1, 1, -1)
        glVertex3f(1, 1, 1); glVertex3f(1, -1, 1)
        
        glEnd()
        glPopMatrix()

    def draw_cylinder(self, radius, height, slices=20):
        """Draw a cylinder"""
        quad = gluNewQuadric()
        gluQuadricNormals(quad, GLU_SMOOTH)
        gluCylinder(quad, radius, radius, height, slices, 1)
        
        # Draw caps
        gluDisk(quad, 0, radius, slices, 1)
        glPushMatrix()
        glTranslatef(0, 0, height)
        gluDisk(quad, 0, radius, slices, 1)
        glPopMatrix()
        gluDeleteQuadric(quad)

    def draw_sphere(self, radius, slices=20, stacks=20):
        """Draw a sphere"""
        quad = gluNewQuadric()
        gluQuadricNormals(quad, GLU_SMOOTH)
        gluSphere(quad, radius, slices, stacks)
        gluDeleteQuadric(quad)

    def draw_wheel(self, x, z, is_front=True):
        """Draw a realistic wheel with proper rotation"""
        glPushMatrix()
        
        # Position wheel
        glTranslatef(x, self.wheel_radius - 0.1, z)
        
        # Apply steering to front wheels
        if is_front:
            glRotatef(self.steering_angle, 0, 1, 0)
        
        # Calculate wheel rotation based on distance traveled
        # The wheel should rotate based on actual movement, not just speed
        glRotatef(self.wheel_rotation, 1, 0, 0)
        
        # Rotate to make cylinder horizontal (along X-axis for side-to-side wheels)
        glPushMatrix()
        glRotatef(90, 0, 1, 0)  # Orient along X-axis
        
        # Draw tire (rubber part)
        glColor3f(*self.colors['wheels'])
        self.draw_cylinder(self.wheel_radius, self.wheel_width, 24)
        
        # Draw rim (metal part) - slightly smaller
        glPushMatrix()
        glTranslatef(0, 0, self.wheel_width * 0.1)
        glColor3f(*self.colors['rims'])
        self.draw_cylinder(self.wheel_radius * 0.6, self.wheel_width * 0.8, 16)
        
        # Draw spokes
        for i in range(5):
            glPushMatrix()
            glRotatef(i * 72, 0, 0, 1)
            glTranslatef(0, 0, self.wheel_width * 0.4)
            glScalef(0.08, self.wheel_radius * 0.5, 0.05)
            glColor3f(*self.colors['rims'])
            self.draw_box(1, 1, 1, self.colors['rims'])
            glPopMatrix()
        
        # Only pop once here since we only pushed once at the start of the wheel rim
        glPopMatrix()
        
        # Pop the matrix for the wheel rotation/orientation
        glPopMatrix()
        
        # Pop the main wheel matrix
        glPopMatrix()

    def draw_car_body(self):
        """Draw realistic car body"""
        # Main body (lower section)
        glPushMatrix()
        glTranslatef(0, self.car_height * 0.25, 0)
        self.draw_box(self.car_width, self.car_height * 0.5, self.car_length, self.colors['body'])
        glPopMatrix()
        
        # Cabin (upper section) - positioned toward rear
        glPushMatrix()
        glTranslatef(0, self.car_height * 0.65, -0.3)
        glScalef(0.85, 0.7, 0.6)
        self.draw_box(self.car_width, self.car_height * 0.5, self.car_length, self.colors['body_dark'])
        glPopMatrix()
        
        # Windshield
        glPushMatrix()
        glTranslatef(0, self.car_height * 0.7, 0.3)
        glRotatef(-15, 1, 0, 0)
        glScalef(0.75, 0.4, 0.5)
        self.draw_box(self.car_width, self.car_height * 0.5, self.car_length, self.colors['windows'])
        glPopMatrix()
        
        # Rear window
        glPushMatrix()
        glTranslatef(0, self.car_height * 0.7, -1.0)
        glRotatef(20, 1, 0, 0)
        glScalef(0.75, 0.35, 0.4)
        self.draw_box(self.car_width, self.car_height * 0.5, self.car_length, self.colors['windows'])
        glPopMatrix()
        
        # Side windows
        for side in [-1, 1]:
            glPushMatrix()
            glTranslatef(side * self.car_width * 0.44, self.car_height * 0.68, 0)
            glScalef(0.05, 0.35, 0.55)
            self.draw_box(self.car_width, self.car_height * 0.5, self.car_length, self.colors['windows'])
            glPopMatrix()
        
        # Hood
        glPushMatrix()
        glTranslatef(0, self.car_height * 0.35, self.car_length * 0.35)
        glScalef(0.85, 0.15, 0.4)
        self.draw_box(self.car_width, self.car_height * 0.5, self.car_length, self.colors['body'])
        glPopMatrix()
        
        # Roof
        glPushMatrix()
        glTranslatef(0, self.car_height * 0.85, -0.2)
        glScalef(0.8, 0.08, 0.5)
        self.draw_box(self.car_width, self.car_height * 0.5, self.car_length, self.colors['body_dark'])
        glPopMatrix()
        
        # Front bumper
        glPushMatrix()
        glTranslatef(0, self.car_height * 0.12, self.car_length * 0.52)
        glScalef(0.95, 0.18, 0.12)
        self.draw_box(self.car_width, self.car_height * 0.5, self.car_length, self.colors['body_dark'])
        glPopMatrix()
        
        # Rear bumper
        glPushMatrix()
        glTranslatef(0, self.car_height * 0.15, -self.car_length * 0.52)
        glScalef(0.95, 0.2, 0.12)
        self.draw_box(self.car_width, self.car_height * 0.5, self.car_length, self.colors['body_dark'])
        glPopMatrix()
        
        # Front grille
        glPushMatrix()
        glTranslatef(0, self.car_height * 0.2, self.car_length * 0.51)
        glScalef(0.6, 0.15, 0.05)
        self.draw_box(self.car_width, self.car_height * 0.5, self.car_length, self.colors['grille'])
        glPopMatrix()
        
        # Headlights
        for side in [-1, 1]:
            glPushMatrix()
            glTranslatef(side * self.car_width * 0.35, self.car_height * 0.25, self.car_length * 0.51)
            glColor3f(*self.colors['headlights'])
            self.draw_sphere(0.15, 12, 12)
            glPopMatrix()
        
        # Taillights
        tail_color = self.colors['brake_lights'] if self.is_braking else self.colors['taillights']
        for side in [-1, 1]:
            glPushMatrix()
            glTranslatef(side * self.car_width * 0.38, self.car_height * 0.25, -self.car_length * 0.51)
            glColor3f(*tail_color)
            self.draw_sphere(0.12, 12, 12)
            glPopMatrix()
        
        # Side mirrors
        for side in [-1, 1]:
            glPushMatrix()
            glTranslatef(side * self.car_width * 0.52, self.car_height * 0.65, 0.8)
            glScalef(0.15, 0.12, 0.18)
            self.draw_box(self.car_width, self.car_height * 0.5, self.car_length, self.colors['body_dark'])
            glPopMatrix()
        
        # Spoiler (rear wing)
        glPushMatrix()
        glTranslatef(0, self.car_height * 0.7, -self.car_length * 0.48)
        # Vertical supports
        for side in [-1, 1]:
            glPushMatrix()
            glTranslatef(side * self.car_width * 0.25, 0, 0)
            glScalef(0.08, 0.3, 0.08)
            self.draw_box(self.car_width, self.car_height * 0.5, self.car_length, self.colors['body_dark'])
            glPopMatrix()
        # Horizontal wing
        glPushMatrix()
        glTranslatef(0, self.car_height * 0.15, 0)
        glScalef(0.9, 0.05, 0.35)
        self.draw_box(self.car_width, self.car_height * 0.5, self.car_length, self.colors['body'])
        glPopMatrix()
        glPopMatrix()

    def steer(self, direction):
        """Improved steering with speed sensitivity"""
        current_speed = math.sqrt(self.velocity[0]**2 + self.velocity[2]**2)
        
        # More responsive at low speeds
        speed_factor = 0.5 + (1.0 - min(1.0, current_speed / (self.max_speed * 0.7))) * 0.5
        
        # Calculate target steering
        target_steer = direction * self.max_steering * speed_factor
        
        # Smooth steering interpolation
        steering_diff = target_steer - self.steering_angle
        steering_speed = 0.2 + min(0.3, abs(steering_diff) / self.max_steering * 0.5)
        
        self.steering_angle += steering_diff * steering_speed
        self.steering_angle = max(-self.max_steering, min(self.max_steering, self.steering_angle))
        
        # Self-centering
        if abs(direction) < 0.1:
            centering_speed = 0.15
            self.steering_angle *= (1.0 - centering_speed)
            if abs(self.steering_angle) < 0.5:
                self.steering_angle = 0

    def update(self):
        """Update car physics and wheel rotation"""
        # Get current speed and direction
        current_speed = math.sqrt(self.velocity[0]**2 + self.velocity[2]**2)
        
        # Apply drag
        drag = 0.98
        self.velocity[0] *= drag
        self.velocity[2] *= drag
        
        # Get forward direction
        rad = math.radians(self.rotation[1])
        forward = [math.sin(rad), 0, math.cos(rad)]
        
        # Apply steering-based turning
        if current_speed > 0.05 and abs(self.steering_angle) > 0.5:
            # Calculate turn rate based on steering angle and speed
            turn_rate = (self.steering_angle / self.max_steering) * self.turn_speed
            turn_rate *= min(1.0, current_speed / self.max_speed)
            
            self.rotation[1] += turn_rate
            
            # Update forward direction after rotation
            rad = math.radians(self.rotation[1])
            forward = [math.sin(rad), 0, math.cos(rad)]
        
        # Update position
        self.position[0] += self.velocity[0]
        self.position[2] += self.velocity[2]
        
        # Update wheel rotation based on actual distance traveled
        distance_moved = math.sqrt(self.velocity[0]**2 + self.velocity[2]**2)
        
        # Calculate rotation: distance / circumference * 360 degrees
        circumference = 2 * math.pi * self.wheel_radius
        rotation_degrees = (distance_moved / circumference) * 360
        
        # Determine direction of rotation
        # Check if moving forward or backward relative to car orientation
        dot_product = forward[0] * self.velocity[0] + forward[2] * self.velocity[2]
        
        if dot_product < 0:  # Moving backward
            rotation_degrees = -rotation_degrees
        
        self.wheel_rotation += rotation_degrees
        self.wheel_rotation %= 360
        
        # Update boost
        if not self.is_boosting and self.boost < self.boost_amount:
            self.boost = min(self.boost_amount, self.boost + self.boost_recharge)
        
        self.speed = current_speed

    def brake(self, strength=1.0):
        """Apply brakes"""
        self.is_braking = True
        current_speed = math.sqrt(self.velocity[0]**2 + self.velocity[2]**2)
        
        if current_speed > 0.01:
            brake_force = min(self.braking * strength, current_speed * 0.3)
            
            if current_speed > 0:
                brake_x = (self.velocity[0] / current_speed) * brake_force
                brake_z = (self.velocity[2] / current_speed) * brake_force
                
                self.velocity[0] -= brake_x
                self.velocity[2] -= brake_z
                
                if abs(self.velocity[0]) < 0.01:
                    self.velocity[0] = 0
                if abs(self.velocity[2]) < 0.01:
                    self.velocity[2] = 0
        else:
            self.velocity[0] = 0
            self.velocity[2] = 0

    def accelerate(self, direction):
        """Accelerate the car"""
        self.is_braking = False
        rad = math.radians(self.rotation[1])
        acceleration = self.acceleration
        
        # Boost effect
        if direction > 0 and self.is_boosting and self.boost > 0:
            acceleration *= self.boost_amount
            self.boost = max(0, self.boost - self.boost_consumption)
        
        # Apply acceleration
        accel_x = math.sin(rad) * acceleration * direction
        accel_z = math.cos(rad) * acceleration * direction
        
        self.velocity[0] += accel_x
        self.velocity[2] += accel_z
        
        # Limit speed
        current_speed = math.sqrt(self.velocity[0]**2 + self.velocity[2]**2)
        max_speed = self.max_speed * (self.boost_amount if self.is_boosting and direction > 0 else 1.0)
        
        if current_speed > max_speed:
            scale = max_speed / current_speed
            self.velocity[0] *= scale
            self.velocity[2] *= scale

    def reset(self):
        """Reset car to initial state"""
        self.position = [0, 0, -10]
        self.rotation = [0, 0, 0]
        self.velocity = [0, 0, 0]
        self.speed = 0
        self.steering_angle = 0
        self.wheel_rotation = 0
        self.is_boosting = False
        self.is_braking = False
        self.boost = 1.0

    def draw(self):
        """Draw the complete car using display lists for better performance"""
        glPushMatrix()
        
        # Apply car position and rotation
        glTranslatef(self.position[0], self.position[1], self.position[2])
        glRotatef(self.rotation[1], 0, 1, 0)
        
        # Draw car body using display list
        glCallList(self.display_lists['body'])
        
        # Draw wheels at correct positions
        front_z = self.wheelbase / 2
        rear_z = -self.wheelbase / 2
        
        # Front right wheel
        glPushMatrix()
        glTranslatef(self.track_width / 2, self.wheel_radius - 0.1, front_z)
        glRotatef(self.steering_angle, 0, 1, 0)
        glRotatef(self.wheel_rotation, 1, 0, 0)
        glRotatef(90, 0, 1, 0)  # Orient along X-axis
        glCallList(self.display_lists['wheel'])
        glPopMatrix()
        
        # Front left wheel
        glPushMatrix()
        glTranslatef(-self.track_width / 2, self.wheel_radius - 0.1, front_z)
        glRotatef(self.steering_angle, 0, 1, 0)
        glRotatef(self.wheel_rotation, 1, 0, 0)
        glRotatef(90, 0, 1, 0)  # Orient along X-axis
        glCallList(self.display_lists['wheel'])
        glPopMatrix()
        
        # Rear right wheel
        glPushMatrix()
        glTranslatef(self.track_width / 2, self.wheel_radius - 0.1, rear_z)
        glRotatef(self.wheel_rotation, 1, 0, 0)
        glRotatef(90, 0, 1, 0)  # Orient along X-axis
        glCallList(self.display_lists['wheel'])
        glPopMatrix()
        
        # Rear left wheel
        glPushMatrix()
        glTranslatef(-self.track_width / 2, self.wheel_radius - 0.1, rear_z)
        glRotatef(self.wheel_rotation, 1, 0, 0)
        glRotatef(90, 0, 1, 0)  # Orient along X-axis
        glCallList(self.display_lists['wheel'])
        glPopMatrix()
        
        glPopMatrix()


class EndlessRoad:
    """Endless scrolling road"""
    def __init__(self):
        self.road_width = 12
        self.lane_width = 4
        self.stripe_length = 3
        self.stripe_gap = 3
        self.tile_size = 50
        
    def draw(self, car_position):
        """Draw road tiles centered around car"""
        glDisable(GL_LIGHTING)
        
        # Calculate which tiles to draw based on car position
        car_z = car_position[2]
        start_tile = int((car_z - 100) / self.tile_size)
        end_tile = int((car_z + 100) / self.tile_size)
        
        for tile_idx in range(start_tile, end_tile + 1):
            tile_z = tile_idx * self.tile_size
            
            # Road surface
            glBegin(GL_QUADS)
            glColor3f(0.15, 0.15, 0.15)  # Dark asphalt
            glVertex3f(-self.road_width/2, 0, tile_z)
            glVertex3f(self.road_width/2, 0, tile_z)
            glVertex3f(self.road_width/2, 0, tile_z + self.tile_size)
            glVertex3f(-self.road_width/2, 0, tile_z + self.tile_size)
            
            # Grass on sides
            glColor3f(0.2, 0.5, 0.2)  # Green grass
            # Left side
            glVertex3f(-self.road_width/2 - 20, -0.05, tile_z)
            glVertex3f(-self.road_width/2, -0.05, tile_z)
            glVertex3f(-self.road_width/2, -0.05, tile_z + self.tile_size)
            glVertex3f(-self.road_width/2 - 20, -0.05, tile_z + self.tile_size)
            # Right side
            glVertex3f(self.road_width/2, -0.05, tile_z)
            glVertex3f(self.road_width/2 + 20, -0.05, tile_z)
            glVertex3f(self.road_width/2 + 20, -0.05, tile_z + self.tile_size)
            glVertex3f(self.road_width/2, -0.05, tile_z + self.tile_size)
            glEnd()
            
            # Center line stripes
            glBegin(GL_QUADS)
            glColor3f(1.0, 1.0, 0.0)  # Yellow center line
            stripe_z = tile_z
            while stripe_z < tile_z + self.tile_size:
                glVertex3f(-0.15, 0.01, stripe_z)
                glVertex3f(0.15, 0.01, stripe_z)
                glVertex3f(0.15, 0.01, stripe_z + self.stripe_length)
                glVertex3f(-0.15, 0.01, stripe_z + self.stripe_length)
                stripe_z += self.stripe_length + self.stripe_gap
            glEnd()
            
            # Edge lines
            glBegin(GL_QUADS)
            glColor3f(1.0, 1.0, 1.0)  # White edge lines
            # Left edge
            glVertex3f(-self.road_width/2 + 0.3, 0.01, tile_z)
            glVertex3f(-self.road_width/2 + 0.5, 0.01, tile_z)
            glVertex3f(-self.road_width/2 + 0.5, 0.01, tile_z + self.tile_size)
            glVertex3f(-self.road_width/2 + 0.3, 0.01, tile_z + self.tile_size)
            # Right edge
            glVertex3f(self.road_width/2 - 0.5, 0.01, tile_z)
            glVertex3f(self.road_width/2 - 0.3, 0.01, tile_z)
            glVertex3f(self.road_width/2 - 0.3, 0.01, tile_z + self.tile_size)
            glVertex3f(self.road_width/2 - 0.5, 0.01, tile_z + self.tile_size)
            glEnd()
        
        glEnable(GL_LIGHTING)


class BluetoothReceiver:
    """Bluetooth controller handler"""
    def __init__(self):
        self.serial_conn = None
        self.command_queue = queue.Queue()
        self.running = False
        self.connected = False
        self.port = None
        self.last_data = {'x': 512, 'y': 512, 'button': 0}
        
    def find_port(self):
        """Find available serial ports"""
        ports = serial.tools.list_ports.comports()
        available_ports = []
        
        print("\n" + "="*60)
        print("Available Serial Ports:")
        print("="*60)
        
        for i, port in enumerate(ports):
            print(f"  [{i}] {port.device} - {port.description}")
            available_ports.append(port.device)
            
            if 'Bluetooth' in port.description or 'HC-0' in port.description:
                print(f"      ^ Bluetooth device detected!")
                self.port = port.device
        
        if not available_ports:
            print("  No serial ports found!")
        
        return available_ports
    
    def connect(self, port=None, baudrate=9600):
        """Connect to serial port"""
        if port:
            self.port = port
        
        if not self.port:
            available = self.find_port()
            if not available:
                return False
            
            if not self.port:
                try:
                    print("\nEnter port number (or 'q' to skip): ", end='')
                    choice = input().strip()
                    if choice.lower() == 'q':
                        return False
                    idx = int(choice)
                    if 0 <= idx < len(available):
                        self.port = available[idx]
                except (ValueError, IndexError):
                    return False
        
        try:
            print(f"\nConnecting to {self.port}...")
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=baudrate,
                timeout=0.1
            )
            time.sleep(2)
            self.connected = True
            print("✓ Connected!\n")
            return True
        except serial.SerialException as e:
            print(f"✗ Failed: {e}")
            return False
    
    def start_listening(self):
        """Start listening thread"""
        if not self.connected:
            return False
        
        self.running = True
        thread = threading.Thread(target=self._listen_loop, daemon=True)
        thread.start()
        return True
    
    def _parse_data(self, line):
        """Parse joystick data"""
        line = line.strip()
        
        # Format: "X:512 Y:512 B:0"
        match = re.search(r'X:?(\d+)\s*Y:?(\d+)\s*B:?(\d+)', line, re.IGNORECASE)
        if match:
            return {
                'x': int(match.group(1)),
                'y': int(match.group(2)),
                'button': int(match.group(3))
            }
        
        # Format: "512,512,0"
        parts = line.split(',')
        if len(parts) >= 3:
            try:
                return {
                    'x': int(parts[0]),
                    'y': int(parts[1]),
                    'button': int(parts[2])
                }
            except ValueError:
                pass
        
        return None
    
    def _listen_loop(self):
        """Listening loop"""
        buffer = ""
        
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        parsed = self._parse_data(line)
                        if parsed:
                            self.last_data = parsed
                            self.command_queue.put(parsed)
            
            except serial.SerialException as e:
                print(f"Serial error: {e}")
                self.connected = False
                break
            except Exception as e:
                print(f"Error: {e}")
            
            time.sleep(0.01)
    
    def get_joystick_state(self):
        """Get current joystick state"""
        return self.last_data.copy()
    
    def stop(self):
        """Stop and close connection"""
        self.running = False
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()


class RacingGame:
    """Main game class"""
    def __init__(self):
        pygame.init()
        self.display = (1366, 768)
        pygame.display.set_mode(self.display, DOUBLEBUF | OPENGL)
        pygame.display.set_caption("Realistic 3D Racing - Enhanced Edition")
        
        # Set up the display mode
        pygame.display.set_mode(self.display, DOUBLEBUF | OPENGL | OPENGLBLIT | HWSURFACE)
        
        # OpenGL setup with error checking
        def check_gl_error(op):
            err = glGetError()
            if err != GL_NO_ERROR:
                error_str = gluErrorString(err)
                error_str = error_str.decode('utf-8') if isinstance(error_str, bytes) else str(error_str)
                print(f"OpenGL error in {op}: {error_str}")
                return False
            return True

        # Set up the viewport
        glViewport(0, 0, self.display[0], self.display[1])
        
        # Set up projection matrix
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(70, (self.display[0] / self.display[1]), 0.1, 1000.0)
        
        # Switch to modelview and set up initial matrix
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        
        # Background color - sky blue
        glClearColor(0.53, 0.81, 0.92, 1.0)
        
        # Enable depth testing and other OpenGL features
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glShadeModel(GL_SMOOTH)
        
        # Lighting setup
        glLightfv(GL_LIGHT0, GL_POSITION, (10, 20, 10, 1))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.6, 0.6, 0.6, 1))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (1.0, 1.0, 1.0, 1))
        glLightfv(GL_LIGHT0, GL_SPECULAR, (0.5, 0.5, 0.5, 1))
        
        # Set up fog for distance
        glEnable(GL_FOG)
        glFogi(GL_FOG_MODE, GL_LINEAR)
        glFogfv(GL_FOG_COLOR, (GLfloat * 4)(0.53, 0.81, 0.92, 1.0))
        glFogf(GL_FOG_DENSITY, 0.1)
        glHint(GL_FOG_HINT, GL_DONT_CARE)
        glFogf(GL_FOG_START, 50.0)
        glFogf(GL_FOG_END, 300.0)
        
        # Check for any OpenGL errors after setup
        if not check_gl_error("OpenGL initialization"):
            print("Warning: OpenGL initialization had errors")
        
        # Game objects
        self.car = RealisticCar()
        self.road = EndlessRoad()
        self.bt_receiver = BluetoothReceiver()
        
        # Game state
        self.clock = pygame.time.Clock()
        self.running = True
        self.font = pygame.font.Font(None, 28)
        
        # Camera settings
        self.camera_modes = ['chase', 'hood', 'far', 'orbit']
        self.camera_mode = 0
        self.camera_distance = 12.0
        self.camera_height = 4.0
        self.camera_position = [0, 4, -12]
        self.camera_target = [0, 0, 0]
        self.orbit_angle = 0
        
        # Control state
        self.joystick_center = 512
        self.deadzone = 100
        self.joystick_history = [(512, 512, 0)] * 3
        
        self.move_forward = False
        self.move_backward = False
        self.turn_left = False
        self.turn_right = False

    def handle_joystick(self, data):
        """Process joystick input"""
        try:
            x = int(data.get('x', self.joystick_center))
            y = int(data.get('y', self.joystick_center))
            button = int(data.get('button', 0))
            
            # Smooth input
            self.joystick_history.append((x, y, button))
            if len(self.joystick_history) > 3:
                self.joystick_history.pop(0)
            
            x = sum(h[0] for h in self.joystick_history) // 3
            y = sum(h[1] for h in self.joystick_history) // 3
            button = 1 if any(h[2] for h in self.joystick_history) else 0
            
            # Process steering (X axis)
            if abs(x - self.joystick_center) < self.deadzone:
                steer_input = 0
            else:
                steer_input = (x - self.joystick_center) / (1023 - self.joystick_center)
                steer_input = math.copysign(abs(steer_input) ** 1.2, steer_input)
                steer_input = max(-1, min(1, steer_input))
            
            # Process acceleration (Y axis)
            if abs(y - self.joystick_center) < self.deadzone:
                accel_input = 0
            else:
                accel_input = (self.joystick_center - y) / (1023 - self.joystick_center)
                accel_input = math.copysign(abs(accel_input) ** 1.2, accel_input)
                accel_input = max(-1, min(1, accel_input))
            
            # Apply inputs
            self.car.steer(steer_input)
            
            if abs(accel_input) > 0.1:
                self.car.accelerate(accel_input)
            else:
                self.car.brake(0.05)
            
            self.car.set_boost(button == 1)
            
            self.move_forward = accel_input > 0.1
            self.move_backward = accel_input < -0.1
            
        except Exception as e:
            print(f"Joystick error: {e}")

    def update_camera(self):
        """Update camera based on mode"""
        mode_name = self.camera_modes[self.camera_mode]
        car_rad = math.radians(self.car.rotation[1])
        
        glLoadIdentity()
        
        if mode_name == 'chase':
            # Chase camera - follows behind car
            distance = 12.0
            height = 4.0
            
            cam_x = self.car.position[0] - math.sin(car_rad) * distance
            cam_y = self.car.position[1] + height
            cam_z = self.car.position[2] - math.cos(car_rad) * distance
            
            look_x = self.car.position[0] + math.sin(car_rad) * 5
            look_y = self.car.position[1] + 1
            look_z = self.car.position[2] + math.cos(car_rad) * 5
            
            gluLookAt(cam_x, cam_y, cam_z, look_x, look_y, look_z, 0, 1, 0)
            
        elif mode_name == 'hood':
            # Hood camera - from driver's perspective
            cam_x = self.car.position[0] + math.sin(car_rad) * 2
            cam_y = self.car.position[1] + 1.5
            cam_z = self.car.position[2] + math.cos(car_rad) * 2
            
            look_x = self.car.position[0] + math.sin(car_rad) * 20
            look_y = self.car.position[1] + 1.0
            look_z = self.car.position[2] + math.cos(car_rad) * 20
            
            gluLookAt(cam_x, cam_y, cam_z, look_x, look_y, look_z, 0, 1, 0)
            
        elif mode_name == 'far':
            # Far camera - overview
            distance = 20.0
            height = 10.0
            
            cam_x = self.car.position[0] - math.sin(car_rad) * distance

    def draw_hud(self):
        """Draw HUD overlay"""
        try:
            # Save OpenGL state
            glDisable(GL_LIGHTING)
            glDisable(GL_DEPTH_TEST)
            
            glMatrixMode(GL_PROJECTION)
            glPushMatrix()
            glLoadIdentity()
            gluOrtho2D(0, self.display[0], self.display[1], 0)
            
            glMatrixMode(GL_MODELVIEW)
            glPushMatrix()
            glLoadIdentity()
            
            # Semi-transparent panel
            glColor4f(0.0, 0.0, 0.0, 0.6)
            glBegin(GL_QUADS)
            glVertex2f(10, 10)
            glVertex2f(350, 10)
            glVertex2f(350, 150)
            glVertex2f(10, 150)
            glEnd()
            
            # Boost bar
            if hasattr(self.car, 'boost'):
                bar_x, bar_y = 10, self.display[1] - 40
                bar_w, bar_h = 250, 25
                
                # Background
                glColor4f(0.2, 0.2, 0.2, 0.8)
                glBegin(GL_QUADS)
                glVertex2f(bar_x, bar_y)
                glVertex2f(bar_x + bar_w, bar_y)
                glVertex2f(bar_x + bar_w, bar_y + bar_h)
                glVertex2f(bar_x, bar_y + bar_h)
                glEnd()
                
                # Boost level
                boost_w = int(bar_w * (self.car.boost / self.car.boost_amount))
                glColor4f(0.0, 0.8, 1.0, 0.9)
                glBegin(GL_QUADS)
                glVertex2f(bar_x, bar_y)
                glVertex2f(bar_x + boost_w, bar_y)
                glVertex2f(bar_x + boost_w, bar_y + bar_h)
                glVertex2f(bar_x, bar_y + bar_h)
                glEnd()
                
                # Border
                glColor4f(1.0, 1.0, 1.0, 0.9)
                glLineWidth(2)
                glBegin(GL_LINE_LOOP)
                glVertex2f(bar_x, bar_y)
                glVertex2f(bar_x + bar_w, bar_y)
                glVertex2f(bar_x + bar_w, bar_y + bar_h)
                glVertex2f(bar_x, bar_y + bar_h)
                glEnd()
            
            # Text info
            bt_status = "Connected" if self.bt_receiver.connected else "Keyboard Only"
            cam_name = self.camera_modes[self.camera_mode].capitalize()
            
            info = [
                f"Speed: {abs(self.car.speed * 100):.0f} km/h",
                f"Boost: {int((self.car.boost/self.car.boost_amount)*100)}%",
                f"Camera: {cam_name}",
                f"Control: {bt_status}",
                "",
                "[C] Camera  [R] Reset"
            ]
            
            for i, line in enumerate(info):
                text = self.font.render(line, True, (255, 255, 255))
                text_data = pygame.image.tostring(text, "RGBA", True)
                glRasterPos2i(20, 25 + i * 22)
                glDrawPixels(text.get_width(), text.get_height(),
                           GL_RGBA, GL_UNSIGNED_BYTE, text_data)
            
            glMatrixMode(GL_MODELVIEW)
            glPopMatrix()
            glMatrixMode(GL_PROJECTION)
            glPopMatrix()
            
            glEnable(GL_DEPTH_TEST)
            glEnable(GL_LIGHTING)
            
        except Exception as e:
            print(f"HUD error: {e}")

    def run(self):
        """Main game loop"""
        # Try Bluetooth connection
        if not self.bt_receiver.connect():
            print("⚠️  Keyboard controls only")
        else:
            self.bt_receiver.start_listening()
            print("✅ Bluetooth ready!")
        
        print("\n=== Controls ===")
        print("Arrow Keys: Drive")
        print("SPACE: Handbrake")
        print("SHIFT: Boost")
        print("C: Change Camera")
        print("R: Reset")
        print("ESC: Quit\n")
        
        while self.running:
            # Handle events
            for event in pygame.event.get():
                if event.type == QUIT:
                    self.running = False
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        self.running = False
                    elif event.key == K_UP:
                        self.move_forward = True
                    elif event.key == K_DOWN:
                        self.move_backward = True
                    elif event.key == K_LEFT:
                        self.turn_left = True
                    elif event.key == K_RIGHT:
                        self.turn_right = True
                    elif event.key in (K_LSHIFT, K_RSHIFT):
                        self.car.set_boost(True)
                    elif event.key == K_SPACE:
                        self.car.drift_factor = 0.8
                    elif event.key == K_c:
                        self.camera_mode = (self.camera_mode + 1) % len(self.camera_modes)
                    elif event.key == K_r:
                        self.car.reset()
                elif event.type == KEYUP:
                    if event.key == K_UP:
                        self.move_forward = False
                    elif event.key == K_DOWN:
                        self.move_backward = False
                    elif event.key == K_LEFT:
                        self.turn_left = False
                    elif event.key == K_RIGHT:
                        self.turn_right = False
                    elif event.key in (K_LSHIFT, K_RSHIFT):
                        self.car.set_boost(False)
                    elif event.key == K_SPACE:
                        self.car.drift_factor = 0.95
            
            # Keyboard controls
            if self.move_forward:
                self.car.accelerate(1)
            elif self.move_backward:
                self.car.accelerate(-1)
            else:
                self.car.brake(0.05)
            
            if self.turn_left:
                self.car.steer(-1)
            elif self.turn_right:
                self.car.steer(1)
            
            # Bluetooth controls
            if self.bt_receiver.connected:
                joystick_data = self.bt_receiver.get_joystick_state()
                self.handle_joystick(joystick_data)
            
            # Update physics
            self.car.update()
            
            # Clear the screen and depth buffer
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            
            # Set up the viewport
            glViewport(0, 0, self.display[0], self.display[1])
            
            # Set up projection matrix
            glMatrixMode(GL_PROJECTION)
            glLoadIdentity()
            gluPerspective(70, (self.display[0] / self.display[1]), 0.1, 1000.0)
            
            # Set up modelview matrix
            glMatrixMode(GL_MODELVIEW)
            glLoadIdentity()
            
            # Update camera position
            self.update_camera()
            
            # Enable depth testing and lighting
            glEnable(GL_DEPTH_TEST)
            glEnable(GL_LIGHTING)
            glEnable(GL_LIGHT0)
            
            # Draw the scene
            self.road.draw(self.car.position)
            self.car.draw()
            
            # Disable lighting for HUD
            glDisable(GL_LIGHTING)
            
            # Draw HUD
            self.draw_hud()
            
            # Re-enable lighting for next frame
            glEnable(GL_LIGHTING)
            
            pygame.display.flip()
            self.clock.tick(60)
        
        # Cleanup
        self.bt_receiver.stop()
        pygame.quit()


if __name__ == '__main__':
    try:
        game = RacingGame()
        game.run()
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
        input("\nPress Enter to exit...")