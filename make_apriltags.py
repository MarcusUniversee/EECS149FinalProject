import cv2
import numpy as np
from reportlab.pdfgen import canvas
from reportlab.lib.units import mm
from reportlab.lib.pagesizes import letter  # or A4
from PIL import Image

# ========= CONFIG =========

# Tag family + ID
TAG_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
TAG_ID = 0

# Side lengths you want ON PAPER (in millimeters)
TAG_SIZES_MM = [30, 40, 50, 60, 70, 80, 90, 100]  # edit as you like

# Page size
PAGE_SIZE = letter  # or use (210*mm, 297*mm) for A4

# Base pixel resolution for the marker image (square)
MARKER_PIXELS = 400  # 400x400 is usually enough


# ========= GENERATE BASE MARKER IMAGE =========

# Generate grayscale AprilTag image (0–255)
marker_img = cv2.aruco.generateImageMarker(TAG_DICT, TAG_ID, MARKER_PIXELS)
# Convert to PIL for saving
marker_pil = Image.fromarray(marker_img)

marker_filename = "apriltag36h11_id0.png"
marker_pil.save(marker_filename)


# ========= BUILD PDF =========

c = canvas.Canvas("apriltag_sizes_tag36h11_id0.pdf", pagesize=PAGE_SIZE)
page_width, page_height = PAGE_SIZE

margin_x = 15 * mm
margin_y = 15 * mm
x = margin_x
y = page_height - margin_y

line_height = 35 * mm  # vertical spacing between tags

c.setFont("Helvetica", 10)

for size_mm in TAG_SIZES_MM:
    # If we run out of space on this page, start a new page
    if y - size_mm * mm < margin_y:
        c.showPage()
        c.setFont("Helvetica", 10)
        x = margin_x
        y = page_height - margin_y

    # Label above the tag
    label = f"Tag36h11, ID=0 — side = {size_mm} mm"
    c.drawString(x, y, label)

    # Draw the marker at EXACT physical size (side length = size_mm)
    # The image will be scaled by reportlab to this width/height.
    c.drawImage(
        marker_filename,
        x,
        y - size_mm * mm - 15 * mm,  # a bit below the label
        width=size_mm * mm,
        height=size_mm * mm,
        preserveAspectRatio=True,
        mask='auto'
    )

    # Move down for the next tag
    y -= (size_mm * mm + line_height)

c.save()

print("Created apriltag_sizes_tag36h11_id0.pdf")
