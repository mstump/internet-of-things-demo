#!/usr/bin/python
"""
This program is demonstration for face and object detection using haar-like features.
The program finds faces in a cap image or video stream and displays a red box around them.

Original C implementation by:  ?
Python implementation by: Roman Stanchak, James Bowman
"""
import sys, os, pygame, time
import cv2, cv2.cv as cv
from optparse import OptionParser

# Parameters for haar detection
# From the API:
# The default parameters (scale_factor=2, min_neighbors=3, flags=0) are tuned
# for accurate yet slow object detection. For a faster operation on real video
# images the settings are:
# scale_factor=1.2, min_neighbors=2, flags=CV_HAAR_DO_CANNY_PRUNING,
# min_size=<minimum possible face size

min_size = (20, 20)
image_scale = 2
haar_scale = 1.2
min_neighbors = 2
haar_flags = 0

def init_display():
    disp_no = os.getenv("DISPLAY")
    drivers = ['fbcon', 'directfb', 'svgalib']
    found = False
    for driver in drivers:
        # Make sure that SDL_VIDEODRIVER is set
        if not os.getenv('SDL_VIDEODRIVER'):
            os.putenv('SDL_VIDEODRIVER', driver)
        try:
            pygame.display.init()
        except pygame.error:
            print 'Driver: {0} failed.'.format(driver)
            continue
        found = True
        break

    if not found:
        raise Exception('No suitable video driver found!')

    size = (pygame.display.Info().current_w, pygame.display.Info().current_h)
    print "Framebuffer size: %d x %d" % (size[0], size[1])
    screen = pygame.display.set_mode(size, pygame.FULLSCREEN)
    pygame.mouse.set_visible(False)
    # Clear the screen to start
    screen.fill((0, 0, 0))
    # Initialise font support
    pygame.font.init()
    # Render the screen
    pygame.display.update()
    return screen, size


def cvimage_to_pygame(image):
    return pygame.image.frombuffer(image.tostring(), cv.GetSize(image), "RGB")


DOWNSCALE = 4

if __name__ == '__main__':

    parser = OptionParser()
    parser.add_option("-c", "--cascade", action="store", dest="cascade", type="str",
                      help="Haar cascade file, default %default",
                      default="../data/haarcascade_frontalface_alt.xml")

    parser.add_option("-z", "--cap", action="store", dest="cap", type="int",
                      help="Camera index, default %default",
                      default=0)
    (options, args) = parser.parse_args()

    classifier = cv2.CascadeClassifier(options.cascade)
    cap        = cv2.VideoCapture(0)
    cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH,960)
    cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT,720)
    # cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 600)
    # cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 800)
    # cap.set(cv2.cv.CV_CAP_PROP_FPS, 30)

    # cap.set(cv2.cv.CV_CAP_PROP_FRAME_WIDTH, 1024)
    # cap.set(cv2.cv.CV_CAP_PROP_FRAME_HEIGHT, 768)
    # time.sleep(2)
    # cap.set(15, -8.0)
    screen, size = init_display()

    i = 0
    while True:
        if not cap.isOpened():
            pass

        rval, frame = cap.read()

        if i % 5 == 0:
            minisize = (frame.shape[1] / DOWNSCALE, frame.shape[0] / DOWNSCALE)
            miniframe = cv2.resize(frame, minisize)
            t = cv.GetTickCount()
            faces = classifier.detectMultiScale(miniframe)
            t = cv.GetTickCount() - t
            for f in faces:
                x, y, w, h = [ v * DOWNSCALE for v in f ]
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255))

            print "detection time = %gms" % (t/(cv.GetTickFrequency()*1000.))

        font = pygame.font.Font(None, 40)
        text_surface = font.render('Faces: %d' % len(faces), True, (255, 255, 255))  # White text
        # Blit the text at 10, 0
        screen.blit(text_surface, (0, 492))

        print frame.shape[0:2]
        screen.blit(pygame.image.frombuffer(frame.tostring(), frame.shape[1::-1], "RGB"), (500, 500))
        pygame.display.update()
        i += 1
