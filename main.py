"""
Drone 2D no Fundo do Windows - Jogo
    Autor: Júlia Onora da Silva
    Data: 6 de março de 2022
"""

import pygame
import time
import numpy as np

from ControleBC import next_step


"""Initializing pygame"""
pygame.init()

# Defining the screen dimension as 800 x 600 pixels
s = (800, 600)  # (x, y) beginning from top left (in pixels)

"""Creating a surface object"""
screen = pygame.display.set_mode(s, pygame.DOUBLEBUF | pygame.HWSURFACE, 32)
# the parameter pygame.HWSURFACE creates a hardware surface,
# what is created using the video card, it was use to acelerate.
# the parameter pygame.DOUBLEBUF acelerate the game:
# it creates two hardware surfaces, but displays just once
# at a time, so, every frame, the program doesn't need to
# generate a new surface, since its already generated.

"""Title and icon"""
pygame.display.set_caption('Drone 2D do Fundo do Windows')  # display the name of the window
icon = pygame.image.load('icons/drone_icon512.png')  # load the icon of the window
pygame.display.set_icon(icon)  # display the icon of the window

"""Windows Background"""
back_img = pygame.image.load('fundo_windows.png')

"""Player"""
drone = 'icons/drone_icon64.png'
playerimg = pygame.image.load(drone)  # load the image

"""Control panel"""
panel_back_img = pygame.image.load('icons\menu_painel.png')

"""Control and sync"""
delta_t = 0.04  # Arbitrary choice: to give enough time for the simulation
estado_inicial = np.array([0., 0.,
                           0., 0.,
                           0., .0,
                           0 * np.pi / 180.,
                           0 * np.pi / 180.])
estado_atual = estado_inicial

"""Control references"""
r_ = np.array([[0., 0.], [0., 10.], [25., 10.], [-30., 2.], [0., 15.], [30., 0.]]).transpose()
r_ID = 0
ref = r_[:, r_ID]

"""Typeface"""
font = pygame.font.Font('Pokemon GB.ttf', 32)
font2 = pygame.font.Font('Pokemon GB.ttf', 16)


"""Functions"""
def meters_to_pixels(xm, ym, s):
    """
    Convert the control script output, in meters, to pixels.
    :param xm: x coordinate in meters
    :param ym: y coordinate in meters
    :param s: screen dimension
    :return: x and y coordinates in pixels
    """
    x_pixels_min = 0
    x_pixels_max = s[0]
    y_pixels_min = s[1]
    y_pixels_max = 0
    x_metros_max = 40
    x_metros_min = -40
    y_metros_max = 18
    y_metros_min = -5
    porc_x = (xm - x_metros_min) / (x_metros_max - x_metros_min)
    porc_y = (ym - y_metros_min) / (y_metros_max - y_metros_min)
    xp = x_pixels_min + ((x_pixels_max - x_pixels_min) * porc_x)
    yp = y_pixels_min + ((y_pixels_max - y_pixels_min) * porc_y)
    return int(xp), int(yp)


def painel(vel, wp):
    """
    Shows everything related to the panel on the screen.
    :param vel: Drone velocity in meters/second
    :param wp: Current waypoint ID
    :return: None
    """
    screen.blit(panel_back_img, (10, 10))
    vel_text = font2.render(f'{vel:.1f}', True, (0, 255, 0))
    wp_text = font.render(f'{wp}', True, (0, 255, 0))
    screen.blit(vel_text, (155, 70))
    screen.blit(wp_text, (300, 75))


def player(x, y, angle):
    """
    Shows the player on the screen.

    Parameters
    ----------
    x : float
        pixel position on x axis (left to right).
    y : float
        pixel position on y axis (top to bottom).
    angle : float
        angle in degrees, positive in counter-clockwise.

    Returns
    -------
    None.

    """
    # generate the rotation by correcting the center of the image
    center = playerimg.get_rect().center  # get the previous center
    image = pygame.transform.rotate(playerimg, angle)  # rotate
    new_rect = image.get_rect(center=center)  # apply the old center
    new_rect = new_rect.move(x, y)  # move the 'rectangle' of the image
    # draw the current image conserving the center and moving the rectangle
    screen.blit(image, new_rect)


def next_ref(ref_vector, current_ref_id):
    """
    Returns the next control reference.
    :param ref_vector: List of all pre-configured references
    :param current_ref_id: Identifier of the current reference
    :return: x and y coordinates of the next reference
    """
    current_ref_id += 1
    if current_ref_id == ref_vector.shape[1]:
        current_ref_id = 0
    return current_ref_id, ref_vector[:, current_ref_id]


"""Buildind the end of the game"""
def end_game():
    """
    Foo structure to easily add an end game routine.
    :return: Boolean to set the end of the game
    """
    return False


"""Buildind the game pre-menu"""
initial_menu = True
while initial_menu:

    # event recognizer
    for event in pygame.event.get():
        # recognize the exit button and end the program
        if event.type == pygame.QUIT:
            pygame.quit()

    # monitoring a key press
    key_pressed = list(pygame.key.get_pressed())
    if True in key_pressed:  # as some key is pressed
        initial_menu = False  # go out the start menu

    # print top texts
    screen.blit(back_img, (0, 0))
    text1 = font.render('Shall we?', True, (255, 255, 255))
    text2 = font2.render('Use SPACE on the keyboard to change the Waypoint', True, (230, 230, 230))
    screen.blit(text1, (30, 30))
    screen.blit(text2, (22, 135))

    # print bottom text
    text3 = font.render('Press any key to start', True, (255, 255, 255))
    screen.blit(text3, (30, 520))

    # shows the screen
    pygame.display.flip()

"""Building the game loop itself"""
running = True
while running:
    inicio_jogo = time.time()

    # event recognizer
    for event in pygame.event.get():
        # recognize the exit button and end the program
        if event.type == pygame.QUIT:
            pygame.quit()

        # Uses KEYDOWN to just consider the pressing of the key, not its hold
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                # update the reference when we press SPACE
                r_ID, ref = next_ref(r_, r_ID)

    # Load the useful information from the response
    xm, ym = estado_atual[2:4]
    vx, vy = estado_atual[4:6]
    vel = (vx**2 + vy**2)**(1/2)
    playerx, playery = meters_to_pixels(xm, ym, s)
    angle = (estado_atual[6]*180)/np.pi

    # if the drone goes further than the boundaries, we send it to the initial spot
    # we subtract 64 from the X and Y limits cause the drone icon have 64 pixels
    if playerx <= 0 or playerx >= s[0] - 64 or playery <= 0 or playery >= s[1] - 64:  # reaching the boundaries
        estado_atual = estado_inicial

    # Build the screen with all elements: background, panel, and drone
    screen.blit(back_img, (0, 0))
    painel(vel=vel, wp=r_ID)
    player(playerx, playery, angle)
    # display everything
    pygame.display.flip()  # using .flip cause of the dual buffer

    # calls the control module to calculate the next state
    proximo_estado = next_step(maxT=delta_t, xo=estado_atual, ref=ref)

    # verify end game
    if end_game():
        running = False  # stops the main loop in case of end game

    # awaits for delta_t, fixing it as the frame rate
    fim_jogo = time.time()
    #print(fim_jogo - inicio_jogo)
    while fim_jogo - inicio_jogo < delta_t/2:
        fim_jogo = time.time()

    # defines the current state as the previous calculated
    estado_atual = proximo_estado
