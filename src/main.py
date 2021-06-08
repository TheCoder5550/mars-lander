#!/usr/bin/env python3

'''

Python version: 3.7.3

'''

import math
import random
from perlin_noise import PerlinNoise
from threading import Timer
import shelve
import pygame

def shelfHasKey(shelf, key):
    try:
        return shelf[key]
    except:
        return False

def defaultKWArg(kwargs, name, defaultValue):
    if name in kwargs:
        return kwargs[name]
    else:
        return defaultValue

def lerp(x, y, t):
    return (1 - t) * x + t * y

def lerpColor(a, b, t):
    return (
        lerp(a[0], b[0], t),
        lerp(a[1], b[1], t),
        lerp(a[2], b[2], t)
    )

def clamp(num, min_value, max_value):
   return max(min(num, max_value), min_value)

def linesIntersect(X, Y, A, B):
  denominator = (Y.x - X.x) * (B.y - A.y) - (Y.y - X.y) * (B.x - A.x)
  epsilon = 0.001
  if (abs(denominator) < epsilon):
    return False

  lambdaNominator = (B.x - A.x) * (X.y - A.y) - (B.y - A.y) * (X.x - A.x)
  lam = lambdaNominator / denominator
  if (lam < 0 or lam > 1):
    return False

  alphaNominator = (Y.y - X.y) * (A.x - X.x) - (Y.x - X.x) * (A.y - X.y)
  alpha = alphaNominator / denominator
  if (alpha < 0 or alpha > 1):
    return False

  return True

def pointInPolygon(point, polygon):
  intersections = 0
  
  for i in range(len(polygon)):
    p = polygon[i]
    p2 = polygon[(i + 1) % len(polygon)]
    
    if (lineToLine(p.x, p.y, p2.x, p2.y, point.x, point.y, point.x + 1e4, point.y)):
        intersections += 1
  
  return intersections % 2 != 0

def lineToLine(p0_x, p0_y, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y):
    s1_x = p1_x - p0_x
    s1_y = p1_y - p0_y
    s2_x = p3_x - p2_x
    s2_y = p3_y - p2_y

    if abs((-s2_x * s1_y + s1_x * s2_y)) < 1e-4:
        return False

    s = (-s1_y * (p0_x - p2_x) + s1_x * (p0_y - p2_y)) / (-s2_x * s1_y + s1_x * s2_y)
    t = ( s2_x * (p0_y - p2_y) - s2_y * (p0_x - p2_x)) / (-s2_x * s1_y + s1_x * s2_y)

    return s >= 0 and s <= 1 and t >= 0 and t <= 1

def lineToLine2(p0, p1, p2, p3):
    return lineToLine(p0.x, p0.y, p1.x, p1.y, p2.x, p2.y, p3.x, p3.y)

class Contact():
    def __init__(self, bodyA, bodyB, pA, pB, normal):
        self.bodyA = bodyA
        self.bodyB = bodyB
        self.pA = pA
        self.pB = pB
        self.normal = normal

def getContactsToTerrain(bodyA, bodyB):
    contacts = []
    pAs = [bodyA.localToWorld(vertex) for vertex in bodyA.vertices]
    pBs = [bodyB.localToWorld(vertex) for vertex in bodyB.vertices]
    normals = []
    for i in range(len(pBs)):
        normals.append(Vector.normal(pBs[i], pBs[(i + 1) % len(pBs)]))

    for pA in pAs:
        if not pointInPolygon(pA, pBs):
            continue

        #pygame.draw.circle(screen, (0, 0, 255), camera.worldToScreen(pA).toInt().toTuple(), 5)

        collisionFace = -1
        for k in range(len(pBs)):
            if lineToLine2(pA, bodyA.position, pBs[k], pBs[(k + 1) % len(pBs)]):
                collisionFace = k
                break
        
        if k == -1:
            continue

        pB = (pA - pBs[collisionFace]).projectOnPlane(normals[collisionFace]) + pBs[collisionFace]

        contacts.append(Contact(bodyA, bodyB, pA, pB, normals[collisionFace]))

    return contacts

class Vector():
    def __init__(self, *args):
        if (isinstance(args[0], tuple)):
            self.x = args[0][0]
            self.y = args[0][1]
        else:
            self.x = args[0]
            self.y = args[1]

    def __str__(self):
        return "x=" + str(self.x) + ", y=" + str(self.y)

    def __add__(self, other):
        return Vector((
            self.x + other.x,
            self.y + other.y
        ))

    def __sub__(self, other):
        return Vector((
            self.x - other.x,
            self.y - other.y
        ))

    def __neg__(self):
        return self * -1

    def __mul__(self, other):
        if isinstance(other, Vector):
            return Vector((
                self.x * other.x,
                self.y * other.y
            ))
        else:
            return Vector((
                self.x * other,
                self.y * other
            ))

    def __truediv__(self, other):
        if isinstance(other, Vector):
            return Vector((
                self.x / other.x,
                self.y / other.y
            ))
        else:
            return Vector((
                self.x / other,
                self.y / other
            ))

    def length(self):
        return math.sqrt(self.x * self.x + self.y * self.y)

    def normalize(self):
        return self / self.length()

    def projectOnPlane(self, n):
        return self - n * Vector.dot(n, self)

    def rotate(v, angle):
        sin = math.sin(angle)
        cos = math.cos(angle)

        return Vector((
            v.x * cos - v.y * sin,
            v.x * sin + v.y * cos
        ))

    def dot(a, b):
        return a.x * b.x + a.y * b.y

    def normal(a, b):
        s = b - a
        return Vector(-s.y, s.x).normalize()

    def cross(a, b):
        return a.x * b.y - a.y * b.x

    def copy(self):
        return Vector(self.x, self.y)

    def toTuple(self):
        return (self.x, self.y)

    def toInt(self):
        return Vector(int(self.x), int(self.y))

class Camera():
    def __init__(self):
        self.position = Vector((0, 0))
        self.scale = 1

    def worldToScreen(self, pos):
        return (Vector(WIDTH, HEIGHT) / 2 + (-self.position + pos) * self.scale * Vector(1, -1)).toInt()

    def worldToScreenFast(self, pos):
        return (Vector(WIDTH, HEIGHT) / 2 + (-self.position + pos) * self.scale * Vector(1, -1))

class ParticleSystem():
    def __init__(self, position):
        self.position = position
        self.particles = []

        self.minHealth = 1.6
        self.maxHealth = 2
        self.particlesPerSecond = 1

        self.emit = True

    def getAcceleration(self, particle):
        return Vector(0, abs(particle.position.x - self.position.x))
    
    def getVelocity(self):
        return Vector((random.random() - 0.5) * 6, 0)
        #return Vector(random.random() - 0.5, -2)

    def update(self, dt):
        if self.emit:
            for _ in range(self.particlesPerSecond):
                p = Particle(self, self.position, self.getVelocity(), random.uniform(self.minHealth, self.maxHealth))
                self.particles.append(p)

        for particle in self.particles:
            particle.update(dt)

    def render(self, camera):
        for particle in self.particles:
            particle.render(camera)

class Particle():
    def __init__(self, parent, position, velocity, health):
        self.parent = parent

        self.position = position
        self.velocity = velocity
        self.startColor = (200, 200, 200)
        self.endColor = (50, 50, 50)
        self.size = 10
        self.gravityScale = 0

        self.maxHealth = health
        self.health = health

    def update(self, dt):
        self.velocity += self.parent.getAcceleration(self) * dt
        self.velocity.y -= GRAVITY * self.gravityScale * dt
        self.position += self.velocity * dt

        self.health -= dt
        if self.health <= 0:
            self.parent.particles.remove(self)

    def render(self, camera):
        sp = camera.worldToScreenFast(self.position)
        t = 1 - self.health / self.maxHealth
        color = lerpColor(self.startColor, self.endColor, t)
        pygame.draw.rect(screen, color, pygame.Rect(sp.x - self.size / 2, sp.y - self.size / 2, self.size, self.size))

class Rigidbody():
    def __init__(self, position, vertices):
        self.static = False
        self.friction = 0.8

        self.mass = 1
        self.position = position
        self.velocity = Vector(0, 0)
        self.force = Vector(0, 0)

        self.inertia = 1
        self.angle = 0
        self.angularVelocity = 0
        self.torque = 0

        self.vertices = vertices

    def update(self, dt):
        if self.static:
            self.velocity = Vector(0, 0)
            self.angularVelocity = 0
        else:
            self.velocity += self.force / self.mass * dt
            self.velocity.y -= GRAVITY * dt
            self.position += self.velocity * dt

            self.angularVelocity += self.torque / self.inertia * dt
            self.angle += self.angularVelocity * dt

            self.force = Vector((0, 0))
            self.torque = 0

    def up(self):
        return Vector.rotate(Vector(0, 1), self.angle)

    def localToWorld(self, pos):
        return self.position + Vector.rotate(pos, self.angle)

    def addForceAtPosition(self, pos, force):
        self.force += force

        r = pos - self.position
        self.torque += r.x * force.y - r.y * force.x

    def render(self, color, camera):
        worldPoints = [camera.worldToScreen(self.localToWorld(vertex)).toTuple() for vertex in self.vertices]
        pygame.draw.polygon(screen, color, worldPoints)

class TargetArrow():
    def __init__(self, target):
        self.target = target
        self.positionOffset = Vector(0, 4)
        self.padding = 75
        self.color = (0, 255, 255)

        self.vertices = [
            Vector(-20, 0),
            Vector(0, 40),
            Vector(20, 0),
            Vector(0, -15)
        ]

    def render(self, camera):
        origSp = camera.worldToScreen(self.target.position + self.positionOffset + Vector(0, 0.5 * math.sin(pygame.time.get_ticks() / 500)))
        sp = origSp.copy()

        sp.x = clamp(sp.x, self.padding, WIDTH - self.padding)
        sp.y = clamp(sp.y, self.padding, HEIGHT - self.padding)
        
        angle = math.atan2(sp.y - origSp.y, sp.x - origSp.x) + math.pi / 2
        if abs(sp.x - origSp.x) < 1e-4 and abs(sp.y - origSp.y) < 1e-4:
            angle = 0

        pygame.draw.polygon(screen, self.color, [(Vector.rotate(x, angle) + sp).toTuple() for x in self.vertices])

class LandingPlatform(Rigidbody):
    def __init__(self, position):
        vertices = [
            Vector(-3, 0),
            Vector(3, 0),
            Vector(3.5, -0.5),
            Vector(3.5, -10),
            Vector(-3.5, -10),
            Vector(-3.5, -0.5)
        ]
        Rigidbody.__init__(self, position, vertices)
        self.static = True

        self.isTarget = False
        self.targetArrow = None

    def render(self, camera):
        Rigidbody.render(self, (100, 100, 100), camera)
        if self.targetArrow:
            self.targetArrow.render(camera)

class Terrain(Rigidbody):
    def __init__(self):
        length = 50
        scale = 4

        position = Vector(-length * scale / 2, -5)

        vertices = []
        for i in range(length):
            vertices.append(Vector(i * scale, noise(i * scale / 200) * 20))

        vertices.append(Vector((length - 1) * scale, -50))
        vertices.append(Vector(0, -50))

        Rigidbody.__init__(self, position, vertices)

        self.static = True

    def getLevel(self, x):
        return noise((x - self.position.x) / 200) * 20 + self.position.y

    def update(self, dt):
        Rigidbody.update(self, dt)

    def localToWorld(self, pos):
        return self.position + pos

    def render(self, camera):
        screenPoints = [camera.worldToScreen(self.position + vertex).toTuple() for vertex in self.vertices]
        screenPoints = [point for point in screenPoints if point[0] > 0 and point[0] > 0 and point[0] < WIDTH and point[1] < HEIGHT]
        
        if len(screenPoints) > 0:
            screenPoints.append((WIDTH, screenPoints[-1][1]))
            screenPoints.append((WIDTH, HEIGHT))
            screenPoints.append((0, HEIGHT))
            screenPoints.append((0, screenPoints[0][1]))

            pygame.draw.polygon(screen, (54, 34, 5), screenPoints)

class Thruster():
    def __init__(self, lander, pos):
        self.lander = lander
        self.position = pos
        self.active = False
        self.mode = 0

        self.vertices = [
            Vector(-0.3, 0),
            Vector(0.3, 0),
            Vector(0, -0.7)
        ]

    def render(self, camera):
        if self.active:
            color1 = (255, 50, 0)
            color2 = (255, 150, 0)
            if self.mode == 1:
                color1 = (0, 0, 255)
                color2 = (100, 100, 255)
            
            worldPoints = [camera.worldToScreen(self.lander.localToWorld(self.position + vertex)).toTuple() for vertex in self.vertices]
            pygame.draw.polygon(screen, color1, worldPoints)

            worldPoints = [camera.worldToScreen(self.lander.localToWorld(self.position + vertex * 0.4)).toTuple() for vertex in self.vertices]
            pygame.draw.polygon(screen, color2, worldPoints)

class Lander(Rigidbody):
    def __init__(self):
        vertices = [
            Vector(-1.4, 1),
            Vector(-1, 1),
            Vector(0, 2),
            Vector(1, 1),
            Vector(1.4, 1),
            Vector(1.6, -1),
            Vector(1.4, -1),
            Vector(2, -2),
            Vector(1.8, -2),
            Vector(1, -1),
            Vector(-1, -1),
            Vector(-1.8, -2),
            Vector(-2, -2),
            Vector(-1.4, -1),
            Vector(-1.6, -1)
        ]

        Rigidbody.__init__(self, Vector(0, 0), vertices)

        self.thrusters = [
            Thruster(self, Vector(1, -0.8)),
            Thruster(self, Vector(-1, -0.8))
        ]

        self.maxFuel = 100
        self.fuel = self.maxFuel

        self.landed = False
        self.landedTimer = 0
        self.touchingGround = False
        self.crashed = False
        self.maxImpulse = 4

        self.landerImage = pygame.image.load(r'./assets/spaceship.png')
        self.landerImage = pygame.transform.scale(self.landerImage, (100, 90))

        self.scoreAngleOffset = 0

    def thrust(self, thrusterIndex, force):
        if self.fuel > 0 and not self.crashed and not self.landed:
            thruster = self.thrusters[thrusterIndex]
            thruster.active = True
            self.addForceAtPosition(self.localToWorld(thruster.position), self.up() * force)

    def update(self, dt):
        if not self.crashed and self.touchingGround and self.velocity.length() < 0.1 and abs(self.angularVelocity) < 0.01:
            self.landedTimer += dt
        else:
            self.landedTimer = 0
        
        if self.landedTimer > 2 and not self.landed:
            scoreHandler.addScore(ScoreHandler.ScoreMessage("Landed", 500))
            self.landed = True
            timer.paused = True
            self.static = True
            
            global savedData
            if not shelfHasKey(savedData, "lowestTime") or timer.time < savedData["lowestTime"]:
                savedData["lowestTime"] = timer.time

            if not shelfHasKey(savedData, "highestScore") or scoreHandler.score > savedData["highestScore"]:
                savedData["highestScore"] = scoreHandler.score

        if not self.crashed and abs(self.angle - self.scoreAngleOffset) > math.pi * 2:
            self.scoreAngleOffset = self.angle
            scoreHandler.addScore(ScoreHandler.ScoreMessage("Flip", 100))

        self.fuel -= self.force.length() * 0.4 * dt
        Rigidbody.update(self, dt)

    def render(self, camera):
        #worldPoints = [camera.worldToScreen(self.position + Vector.rotate(vertex, self.angle)).toTuple() for vertex in self.vertices]
        #pygame.draw.polygon(screen, (0, 255, 0), worldPoints)

        p = camera.worldToScreen(self.position) - Vector(self.landerImage.get_width(), self.landerImage.get_height()) / 2
        rotatedImage = pygame.transform.rotate(self.landerImage, self.angle / math.pi * 180)
        newRect = rotatedImage.get_rect(center = self.landerImage.get_rect(topleft = p.toInt().toTuple()).center)
        screen.blit(rotatedImage, newRect)

        for thruster in self.thrusters:
            thruster.render(camera)

class ScoreHandler():
    class ScoreMessage():
        def __init__(self, message, score, currentCombo = 1):
            self.message = message
            self.score = score
            self.currentCombo = currentCombo

    def __init__(self):
        self.score = 0
        self.scoreHistory = []
        self.yOffset = 0

    def removeScore(self, scoreMessage):
        try:
            self.scoreHistory.remove(scoreMessage)
        except:
            pass

    def addScore(self, scoreMessage):
        scoreMessage.currentCombo = 2 if len(self.scoreHistory) > 0 else 1
        self.score += scoreMessage.score * scoreMessage.currentCombo
        self.scoreHistory.append(scoreMessage)

        self.yOffset = 1
        Timer(5.0, self.removeScore, (scoreMessage,)).start()

    def update(self, dt):
        self.yOffset -= 4 * dt
        if self.yOffset < 0:
            self.yOffset = 0

    def render(self, position):
        i = 0
        for history in reversed(self.scoreHistory):
            output = "+" + str(history.score)
            if history.currentCombo != 1:
                output += "x" + str(history.currentCombo)
            output += " " + history.message
            
            drawText(output, (position + Vector(0, (i - self.yOffset) * 30)).toTuple(), WHITE if i == 0 else (100, 100, 100))
            i += 1

class GameTimer():
    def __init__(self):
        self.timeOffset = pygame.time.get_ticks()
        self.time = 0
        self.paused = False

    def reset(self):
        self.time = 0
        self.timeOffset = pygame.time.get_ticks()

    def getFormattedTime(time):
        totalSeconds = int(time / 1000)
        minutes = int(totalSeconds / 60)
        seconds = totalSeconds - minutes * 60
        millis = int(time) % 1000
        return "{:02d}:{:02d}:{:03d}".format(minutes, seconds, millis)

    def update(self, dt):
        if not self.paused:
            self.time += dt * 1000
            #self.time = pygame.time.get_ticks() - self.timeOffset

    def render(self, position):
        textRender = gameFont.render(GameTimer.getFormattedTime(self.time), True, WHITE)
        rt = textRender.get_rect()
        screen.blit(textRender, dest=(position[0] - rt.width / 2, position[1] - rt.height / 2))

def solveCollisions(dt):
    lander.touchingGround = False

    contacts = []
    for a in gameObjects:
        for b in gameObjects:
            if a != b:
                contacts += getContactsToTerrain(a, b)
    
    lambdaAccumulated = [0] * len(contacts)
    for j in range(constraintIterations):
        for i in range(len(contacts)):
            c = contacts[i]

            #pygame.draw.circle(screen, (0, 0, 255), camera.worldToScreen(c.pA).toInt().toTuple(), 5)
            #pygame.draw.circle(screen, (255, 0, 255), camera.worldToScreen(c.pB).toInt().toTuple(), 3)

            if not math.isnan(c.pA.x) and not math.isnan(c.pB.x) and not (c.bodyA.static and c.bodyB.static):
                bodyAMult = 0 if c.bodyA.static else 1
                bodyBMult = 0 if c.bodyB.static else 1

                jacobian = [
                    c.normal.x * bodyAMult,
                    c.normal.y * bodyAMult,
                    Vector.cross(c.pA - c.bodyA.position, c.normal) * bodyAMult,
                    -c.normal.x * bodyBMult,
                    -c.normal.y * bodyBMult,
                    -Vector.cross(c.pB - c.bodyB.position, c.normal) * bodyBMult
                ]

                tangent = Vector(-c.normal.y, c.normal.x)

                jacobianFriction = [
                    tangent.x * bodyAMult,
                    tangent.y * bodyAMult,
                    Vector.cross(c.pA - c.bodyA.position, tangent) * bodyAMult,
                    -tangent.x * bodyBMult,
                    -tangent.y * bodyBMult,
                    -Vector.cross(c.pB - c.bodyB.position, tangent) * bodyBMult
                ]

                velocityMatrix = [
                    c.bodyA.velocity.x,
                    c.bodyA.velocity.y,
                    c.bodyA.angularVelocity,
                    c.bodyB.velocity.x,
                    c.bodyB.velocity.y,
                    c.bodyB.angularVelocity
                ]

                masses = [
                    c.bodyA.mass,
                    c.bodyA.mass,
                    c.bodyA.inertia,
                    c.bodyB.mass,
                    c.bodyB.mass,
                    c.bodyB.inertia
                ]

                C = Vector.dot(c.pA - c.pB, c.normal)
                bias = 0.5 / dt * (C if C < 0 else 0)

                JVSum = sum([j * v for j, v in zip(jacobian, velocityMatrix)])
                denom = sum([j * j / m for j, m in zip(jacobian, masses)])

                λ = -(JVSum + bias) / denom

                if lambdaAccumulated[i] + λ < 0:
                    λ = -lambdaAccumulated[i]

                lambdaAccumulated[i] += λ

                if c.bodyA == lander or c.bodyB == lander:
                    if (isinstance(c.bodyA, LandingPlatform) and c.bodyA.isTarget) or (isinstance(c.bodyB, LandingPlatform) and c.bodyB.isTarget):
                        lander.touchingGround = True
                    if λ > lander.maxImpulse:
                        lander.crashed = True

                c.bodyA.velocity.x += jacobian[0] * λ / c.bodyA.mass
                c.bodyA.velocity.y += jacobian[1] * λ / c.bodyA.mass
                c.bodyA.angularVelocity += jacobian[2] * λ / c.bodyA.inertia
                c.bodyB.velocity.x += jacobian[3] * λ / c.bodyB.mass
                c.bodyB.velocity.y += jacobian[4] * λ / c.bodyB.mass
                c.bodyB.angularVelocity += jacobian[5] * λ / c.bodyB.inertia

                JVSum = sum([j * v for j, v in zip(jacobianFriction, velocityMatrix)])
                denom = sum([j * j / m for j, m in zip(jacobianFriction, masses)])

                friction = c.bodyA.friction * c.bodyB.friction
                λFriction = -JVSum / denom
                λFriction = clamp(λFriction, -friction * λ, friction * λ)

                c.bodyA.velocity.x += jacobianFriction[0] * λFriction / c.bodyA.mass
                c.bodyA.velocity.y += jacobianFriction[1] * λFriction / c.bodyA.mass
                c.bodyA.angularVelocity += jacobianFriction[2] * λFriction / c.bodyA.inertia
                c.bodyB.velocity.x += jacobianFriction[3] * λFriction / c.bodyB.mass
                c.bodyB.velocity.y += jacobianFriction[4] * λFriction / c.bodyB.mass
                c.bodyB.angularVelocity += jacobianFriction[5] * λFriction / c.bodyB.inertia

def drawText(text, pos, color):
    textRender = gameFont.render(text, True, color)
    screen.blit(textRender, dest=pos)

def drawUI():
    drawText("FPS: " + str(int(clock.get_fps())), (WIDTH - 90, 10), WHITE)

    drawText("FUEL", (20, 20), WHITE)

    h = clamp(lander.fuel / lander.maxFuel, 0, 1) * 100
    if h > 0:
        pygame.draw.rect(screen, WHITE, pygame.Rect(35, 140 - h, 10, h))

    if lander.fuel < lander.maxFuel * 0.3:
        t = pygame.time.get_ticks() / 1000
        if t % 1 < 0.5:
            drawText("WARNING", (20, 150), (255, 0, 0))

    drawText("ELEVATION: " + str(round(lander.position.y, 2)), (WIDTH - 200, 50), WHITE)

    if lander.crashed:
        drawText("CRASHED", (20, 200), (255, 0, 0))

    if lander.landed:
        drawText("LANDED", (20, 240), (0, 200, 0))

    timer.render((WIDTH / 2, 25))

    textRender = gameFont.render("SCORE: " + str(scoreHandler.score), True, WHITE)
    rt = textRender.get_rect()
    screen.blit(textRender, dest=(WIDTH / 2 - rt.width / 2, 60 - rt.height / 2))

    scoreHandler.render(Vector(WIDTH / 2 + 70, HEIGHT / 2 - 70))

    if paused:
        s = pygame.Surface((WIDTH, HEIGHT))  # the size of your rect
        s.set_alpha(100)                # alpha level
        s.fill((0,0,0))           # this fills the entire surface
        screen.blit(s, (0,0))    # (0,0) are the top-left coordinates

        textRender = gameFont.render("PAUSED", True, WHITE)
        rt = textRender.get_rect()
        screen.blit(textRender, dest=(WIDTH / 2 - rt.width / 2, HEIGHT / 2 - rt.height / 2))

class PygameButton():
    def __init__(self, screen, rect, text, **kwargs):
        self.screen = screen
        self.rect = rect
        self.borderSize = defaultKWArg(kwargs, "borderSize", 5)
        self.setBorderRect()
        self.text = text

        self.borderColor = defaultKWArg(kwargs, "borderColor", (255, 0, 0))
        self.pressedBorderColor = defaultKWArg(kwargs, "pressedBorderColor", self.borderColor)
        self.currentBorderColor = self.borderColor

        self.backgroundColor = defaultKWArg(kwargs, "backgroundColor", (0, 255, 0))
        self.pressedBackgroundColor = defaultKWArg(kwargs, "pressedBackgroundColor", tuple(0.5 * x for x in self.backgroundColor))
        self.currentBackgroundColor = self.backgroundColor

        self.textColor = defaultKWArg(kwargs, "textColor", (0, 0, 0))
        self.pressedTextColor = defaultKWArg(kwargs, "pressedTextColor", self.textColor)
        self.currentTextColor = self.textColor

        self.visible = True

        self.onClick = lambda x : None

    def setBorderRect(self):
        self.borderRect = pygame.Rect(self.rect.x + self.borderSize, self.rect.y + self.borderSize, self.rect.width - 2 * self.borderSize, self.rect.height - 2 * self.borderSize)

    def inside(self, mouse):
        return self.rect.left <= mouse[0] <= self.rect.right and self.rect.top <= mouse[1] <= self.rect.bottom

    def mousedown(self, mouse):
        if self.visible and self.inside(mouse):
            self.currentBackgroundColor = self.pressedBackgroundColor
            self.currentBorderColor = self.pressedBorderColor
            self.currentTextColor = self.pressedTextColor

    def mouseup(self, mouse):
        self.currentBackgroundColor = self.backgroundColor
        self.currentBorderColor = self.borderColor
        self.currentTextColor = self.textColor

        if self.visible and self.inside(mouse):
            self.onClick(self)

    def render(self):
        if self.visible:
            pygame.draw.rect(self.screen, self.currentBorderColor, self.rect)
            pygame.draw.rect(self.screen, self.currentBackgroundColor, self.borderRect)

            textRender = gameFont.render(self.text, True, self.currentTextColor)
            rt = textRender.get_rect()
            self.screen.blit(textRender, dest=(self.rect.x + self.rect.width / 2 - rt.width / 2, self.rect.y + self.rect.height / 2 - rt.height / 2))

def startGame(b):
    global menu
    menu = False
    b.visible = False
    timer.reset()

    resetGame()

def resetGame():
    global menu, paused

    paused = False

    lander.static = False
    lander.position = Vector(0, -1.9)
    lander.velocity = Vector(0, 0)
    lander.angle = 0
    lander.angularVelocity = 0
    lander.crashed = False
    lander.landed = False
    lander.landedTimer = 0
    lander.fuel = lander.maxFuel
    lander.touchingGround = False
    lander.scoreAngleOffset = 0

    dust.particles = []
    scoreHandler.scoreHistory = []
    scoreHandler.score = 0
    timer.reset()
    timer.paused = False

    gotoMenuButton.visible = False

def gotoMenu(_):
    global menu
    menu = True
    startButton.visible = True
    gotoMenuButton.visible = False

def update():
    startButton.rect = pygame.Rect(WIDTH / 2 - 175 / 2, HEIGHT / 2 - 75 / 2, 175, 75)
    startButton.setBorderRect()

    gotoMenuButton.rect = pygame.Rect(WIDTH / 2 - 50, HEIGHT / 2 + 50, 100, 50)
    gotoMenuButton.setBorderRect()

    if not menu:
        if not paused:
            keys = pygame.key.get_pressed()

            thrusting = False
            if keys[pygame.K_d]:
                lander.thrust(0, 7)
                thrusting = True
            else:
                lander.thrusters[0].active = False

            if keys[pygame.K_a]:
                lander.thrust(1, 7)
                thrusting = True
            else:
                lander.thrusters[1].active = False

            if keys[pygame.K_SPACE]:
                lander.thrust(0, 15)
                lander.thrust(1, 15)
                lander.thrusters[0].mode = 1
                lander.thrusters[1].mode = 1
                thrusting = True
            else:
                lander.thrusters[0].mode = 0
                lander.thrusters[1].mode = 0

            y = terrain.getLevel(lander.position.x)
            dust.emit = thrusting and lander.fuel > 0 and not lander.crashed and not lander.landed and lander.position.y - y < 7
            dust.position = Vector(lander.position.x, y)
            dust.update(fixedDeltaTime)

            for obj in gameObjects:
                obj.update(fixedDeltaTime)

            scoreHandler.update(fixedDeltaTime)
            timer.update(fixedDeltaTime)

            camera.position = lander.position

            solveCollisions(fixedDeltaTime)

def render():
    screen.fill(BACKGROUND_COLOR)
    
    if menu:
        textRender = titleFont.render(GAME_TITLE.upper(), True, WHITE)
        rt = textRender.get_rect()
        screen.blit(textRender, dest=(WIDTH / 2 - rt.width / 2, HEIGHT / 2 - 125 - rt.height / 2))

        textRender = gameFont.render("BEST TIME", True, WHITE)
        rt = textRender.get_rect()
        screen.blit(textRender, dest=(WIDTH / 2 - rt.width / 2, HEIGHT / 2 + 125 - rt.height / 2))

        highscore = "---" if not shelfHasKey(savedData, "lowestTime") else GameTimer.getFormattedTime(savedData["lowestTime"])
        textRender = gameFont.render(highscore, True, WHITE)
        rt = textRender.get_rect()
        screen.blit(textRender, dest=(WIDTH / 2 - rt.width / 2, HEIGHT / 2 + 160 - rt.height / 2))

        textRender = gameFont.render("HIGHSCORE", True, WHITE)
        rt = textRender.get_rect()
        screen.blit(textRender, dest=(WIDTH / 2 - rt.width / 2, HEIGHT / 2 + 225 - rt.height / 2))

        highscore = "---" if not shelfHasKey(savedData, "highestScore") else str(savedData["highestScore"])
        textRender = gameFont.render(highscore, True, WHITE)
        rt = textRender.get_rect()
        screen.blit(textRender, dest=(WIDTH / 2 - rt.width / 2, HEIGHT / 2 + 260 - rt.height / 2))
    else:
        for obj in gameObjects:
            obj.render(camera)
        dust.render(camera)

        #p = camera.worldToScreen(Vector(-4, -6)).toTuple()
        #screen.blit(controlsImage, p)

        drawUI()

    for button in buttons:
        button.render()

# Settings
GAME_TITLE = "Mars lander"
WIDTH = 800
HEIGHT = 600
BACKGROUND_COLOR = (10, 5, 0)
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

constraintIterations = 4
fixedDeltaTime = 1 / 60
GRAVITY = 3.72 # mars gravity

# Init
savedData = shelve.open('score')

pygame.init()
pygame.font.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)
pygame.display.set_caption(GAME_TITLE)

clock = pygame.time.Clock()
gameFont = pygame.font.SysFont(pygame.font.get_default_font(), 30)
titleFont = pygame.font.SysFont(pygame.font.get_default_font(), 100)

noise = PerlinNoise(octaves=10, seed=2)

startButton = PygameButton(screen, pygame.Rect(WIDTH / 2 - 175 / 2, HEIGHT / 2 - 75 / 2, 175, 75), "START GAME", textColor=WHITE, borderColor=WHITE, backgroundColor=BLACK, pressedTextColor=BLACK, pressedBackgroundColor=WHITE)
startButton.onClick = startGame

gotoMenuButton = PygameButton(screen, pygame.Rect(WIDTH / 2 - 50, HEIGHT / 2 + 50, 100, 50), "MENU", textColor=WHITE, borderColor=WHITE, backgroundColor=BLACK, pressedTextColor=BLACK, pressedBackgroundColor=WHITE)
gotoMenuButton.onClick = gotoMenu
gotoMenuButton.visible = False

buttons = [startButton, gotoMenuButton]

camera = Camera()
camera.scale = 20

lander = Lander()
lander.position.y = -1.9
lander.inertia = 4

terrain = Terrain()

dust = ParticleSystem(Vector(0, 0))

gameObjects = [
    LandingPlatform(Vector(0, -4)),
    LandingPlatform(Vector(80, max(terrain.getLevel(80 - 3.5), terrain.getLevel(80 + 3.5)) + 1)),
    terrain,
    lander,
]
gameObjects[1].isTarget = True
gameObjects[1].targetArrow = TargetArrow(gameObjects[1])

scoreHandler = ScoreHandler()
timer = GameTimer()

controlsImage = pygame.image.load(r'./assets/spaceship.png')
controlsImage = pygame.transform.scale(controlsImage, (150, 150))

menu = True
paused = False
running = True

while running:
    mouse = pygame.mouse.get_pos()

    # Events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_ESCAPE and not menu:
                paused = not paused
                gotoMenuButton.visible = paused
            if event.key == pygame.K_r:
                resetGame()
            if event.key == pygame.K_m:
                gotoMenu(gotoMenuButton)

        for button in buttons:
            if event.type == pygame.MOUSEBUTTONDOWN:
                button.mousedown(mouse)

            if event.type == pygame.MOUSEBUTTONUP:
                button.mouseup(mouse)

        if event.type == pygame.VIDEORESIZE:
            WIDTH = event.w
            HEIGHT = event.h
            screen = pygame.display.set_mode((WIDTH, HEIGHT), pygame.RESIZABLE)

    update()
    render()

    # Render to screen
    pygame.display.flip()
    clock.tick(60)

savedData.close()
pygame.quit()