"""
3D Mercedes-Benz Car Driving Game with Bluetooth Joystick Control
---------------------------------------------------------------
Requirements:
    pip install pygame PyOpenGL pyserial

Controls:
    - Arrow keys or Bluetooth joystick
    - Up: move forward
    - Down: move backward
    - Left/Right: steer
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


# =============================
# Bluetooth Receiver (same as before)
# =============================
class BluetoothReceiver:
    def __init__(self, baudrate=38400):
        self.serial_conn = None
        self.baudrate = baudrate
        self.thread = None
        self.running = False
        self.command_queue = queue.Queue()

    def find_port(self):
        ports = serial.tools.list_ports.comports()
        for port in ports:
            if "HC-05" in port.description or "HC-06" in port.description or "Bluetooth" in port.description:
                return port.device
        return None

    def connect(self):
        try:
            port = self.find_port()
            if not port:
                print("⚠️ No Bluetooth port found. Try pairing HC-05 first.")
                return False
            self.serial_conn = serial.Serial(port, self.baudrate, timeout=1)
            print(f"✅ Connected to {port}")
            return True
        except Exception as e:
            print("Bluetooth connection error:", e)
            return False

    def start_listening(self):
        if not self.serial_conn:
            return
        self.running = True
        self.thread = threading.Thread(target=self._listen_loop, daemon=True)
        self.thread.start()

    def _listen_loop(self):
        while self.running:
            try:
                line = self.serial_conn.readline().decode().strip()
                if line:
                    self.command_queue.put(line)
            except Exception:
                break

    def get_command(self):
        try:
            return self.command_queue.get_nowait()
        except queue.Empty:
            return None

    def stop(self):
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()


# =============================
# Car Class (Mercedes-Benz)
# =============================
class MercedesCar:
    def __init__(self):
        self.position = [0, 0, 0]
        self.angle = 0
        self.speed = 0
        self.max_speed = 0.3
        self.turn_speed = 2
        self.acceleration = 0.01

    def draw(self):
        glPushMatrix()
        glTranslatef(self.position[0], self.position[1] + 0.5, self.position[2])
        glRotatef(self.angle, 0, 1, 0)

        # --- Car body ---
        glColor3f(0.5, 0.5, 0.55)  # Metallic gray
        glPushMatrix()
        glScalef(2.5, 0.5, 5)
        self.draw_cube()
        glPopMatrix()

        # --- Windows ---
        glColor3f(0.3, 0.35, 0.4)
        glPushMatrix()
        glTranslatef(0, 0.4, 0)
        glScalef(1.8, 0.3, 2)
        self.draw_cube()
        glPopMatrix()

        # --- Headlights ---
        glColor3f(1, 1, 0.9)
        for side in (-0.9, 0.9):
            glPushMatrix()
            glTranslatef(side, 0.1, -2.6)
            self.draw_sphere(0.15)
            glPopMatrix()

        # --- Wheels ---
        glColor3f(0.1, 0.1, 0.1)
        for dx, dz in [(-1.2, 2), (1.2, 2), (-1.2, -2), (1.2, -2)]:
            glPushMatrix()
            glTranslatef(dx, -0.3, dz)
            glRotatef(90, 0, 1, 0)
            self.draw_cylinder(0.4, 0.2)
            glPopMatrix()

        glPopMatrix()

    def draw_cube(self):
        glBegin(GL_QUADS)
        # Front
        glVertex3f(-1, -1, 1); glVertex3f(1, -1, 1)
        glVertex3f(1, 1, 1); glVertex3f(-1, 1, 1)
        # Back
        glVertex3f(-1, -1, -1); glVertex3f(-1, 1, -1)
        glVertex3f(1, 1, -1); glVertex3f(1, -1, -1)
        # Top
        glVertex3f(-1, 1, -1); glVertex3f(-1, 1, 1)
        glVertex3f(1, 1, 1); glVertex3f(1, 1, -1)
        # Bottom
        glVertex3f(-1, -1, -1); glVertex3f(1, -1, -1)
        glVertex3f(1, -1, 1); glVertex3f(-1, -1, 1)
        # Left
        glVertex3f(-1, -1, -1); glVertex3f(-1, -1, 1)
        glVertex3f(-1, 1, 1); glVertex3f(-1, 1, -1)
        # Right
        glVertex3f(1, -1, -1); glVertex3f(1, 1, -1)
        glVertex3f(1, 1, 1); glVertex3f(1, -1, 1)
        glEnd()

    def draw_cylinder(self, radius, height):
        quad = gluNewQuadric()
        gluCylinder(quad, radius, radius, height, 20, 20)

    def draw_sphere(self, r):
        quad = gluNewQuadric()
        gluSphere(quad, r, 20, 20)

    def update(self):
        # Move forward in facing direction
        rad = math.radians(self.angle)
        self.position[0] += math.sin(rad) * self.speed
        self.position[2] += math.cos(rad) * self.speed

    def steer(self, direction):
        self.angle += direction * self.turn_speed

    def accelerate(self, direction):
        self.speed += direction * self.acceleration
        self.speed = max(-self.max_speed, min(self.max_speed, self.speed))

    def reset(self):
        self.position = [0, 0, 0]
        self.angle = 0
        self.speed = 0


# =============================
# Game Viewer
# =============================
class DrivingGame:
    def __init__(self):
        pygame.init()
        self.display = (1000, 700)
        pygame.display.set_mode(self.display, DOUBLEBUF | OPENGL)
        pygame.display.set_caption("🚗 Mercedes-Benz Driving Game")

        # Set up the projection matrix
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45, (self.display[0]/self.display[1]), 0.1, 1000.0)
        
        # Switch to modelview matrix
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        
        # Set up the camera
        self.camera_distance = 10.0
        self.camera_height = 3.0
        self.camera_angle = 0.0
        
        # Enable depth testing and lighting
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glLightfv(GL_LIGHT0, GL_POSITION, (5, 10, 5, 1))
        glLightfv(GL_LIGHT0, GL_DIFFUSE, (1, 1, 1, 1))
        glLightfv(GL_LIGHT0, GL_AMBIENT, (0.2, 0.2, 0.2, 1))
        
        # Enable color material for proper lighting on colored objects
        glEnable(GL_COLOR_MATERIAL)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        
        # Set a nice background color (sky blue)
        glClearColor(0.53, 0.81, 0.92, 1.0)

        self.car = MercedesCar()
        self.bluetooth = BluetoothReceiver()
        self.clock = pygame.time.Clock()

    def draw_road(self):
        # Road base
        glColor3f(0.2, 0.2, 0.2)
        glBegin(GL_QUADS)
        glVertex3f(-4, -0.5, 100)
        glVertex3f(4, -0.5, 100)
        glVertex3f(4, -0.5, -100)
        glVertex3f(-4, -0.5, -100)
        glEnd()

        # Road borders (green grass)
        glColor3f(0.1, 0.5, 0.1)
        for offset in (-6, 6):
            glBegin(GL_QUADS)
            glVertex3f(offset - 2, -0.5, 100)
            glVertex3f(offset + 2, -0.5, 100)
            glVertex3f(offset + 2, -0.5, -100)
            glVertex3f(offset - 2, -0.5, -100)
            glEnd()

        # Lane markings
        glColor3f(1, 1, 1)
        glBegin(GL_LINES)
        for z in range(-100, 100, 5):
            glVertex3f(0, -0.49, z)
            glVertex3f(0, -0.49, z + 2)
        glEnd()

    def handle_command(self, cmd):
        if cmd == "forward":
            self.car.accelerate(1)
        elif cmd == "backward":
            self.car.accelerate(-1)
        elif cmd == "left":
            self.car.steer(1)
        elif cmd == "right":
            self.car.steer(-1)
        elif cmd == "reset":
            self.car.reset()

    def run(self):
        print("Keyboard Controls: ↑ ↓ ← →, R = Reset, Esc = Quit")
        self.bluetooth.connect()
        self.bluetooth.start_listening()

        while True:
            for event in pygame.event.get():
                if event.type == QUIT:
                    self.bluetooth.stop()
                    pygame.quit()
                    return

            keys = pygame.key.get_pressed()
            if keys[K_ESCAPE]:
                self.bluetooth.stop()
                pygame.quit()
                return
            if keys[K_UP]:
                self.car.accelerate(1)
            if keys[K_DOWN]:
                self.car.accelerate(-1)
            if keys[K_LEFT]:
                self.car.steer(1)
            if keys[K_RIGHT]:
                self.car.steer(-1)
            if keys[K_r]:
                self.car.reset()

            # Bluetooth command
            cmd = self.bluetooth.get_command()
            if cmd:
                self.handle_command(cmd)

            self.car.update()

            # Clear the screen and depth buffer
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            
            # Reset the modelview matrix
            glLoadIdentity()
            
            # Position the camera behind and above the car
            camera_x = self.car.position[0] - math.sin(math.radians(self.car.angle)) * self.camera_distance
            camera_z = self.car.position[2] - math.cos(math.radians(self.car.angle)) * self.camera_distance
            camera_y = self.car.position[1] + self.camera_height
            
            # Look at a point in front of the car
            look_x = self.car.position[0] + math.sin(math.radians(self.car.angle))
            look_z = self.car.position[2] + math.cos(math.radians(self.car.angle))
            look_y = self.car.position[1] + 1.0
            
            # Apply the camera transformation
            gluLookAt(camera_x, camera_y, camera_z,
                     look_x, look_y, look_z,
                     0, 1, 0)
            
            # Draw the scene
            self.draw_road()
            self.car.draw()
            
            # Update the display
            pygame.display.flip()
            self.clock.tick(60)


if __name__ == "__main__":
    DrivingGame().run()
