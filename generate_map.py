html = ""
rooms = [
    # Top left loop left edge (labels on left)
    ("6-105", 1450, 1750, "right: 60px;"),
    ("6-106", 1600, 1750, "right: 60px;"),
    ("6-107", 1750, 1750, "right: 60px;"),
    ("6-108", 1850, 1750, "right: 60px;"),
    
    # Top left loop inner right edge (labels on right)
    ("6-104", 1500, 1900, "left: 60px;"),
    ("6-109", 1800, 1900, "left: 60px;"),
    
    # Top left loop inner left edge (labels on left)
    ("6-102", 1550, 2050, "right: 60px;"),
    ("6-101", 1650, 2050, "right: 60px;"),
    
    # Top right area
    ("6-103", 1450, 2400, "right: 60px;"),
    
    # Lower vertical hall left edge
    ("6PC1", 2200, 2150, "right: 60px;"),
    ("6-112", 2500, 2150, "right: 60px;"),
    ("6-113", 2750, 2150, "right: 60px;"),
    
    # Lower vertical hall right edge (labels on right)
    ("6-1S1", 2100, 2350, "right: 60px;"),
    ("6-111", 2350, 2300, "left: 60px;"),
    ("6-114", 2800, 2300, "left: 60px;"),
    ("6-115", 2900, 2350, "left: 60px;")
]

for name, top, left, label_pos in rooms:
    html += f'<div class="room-btn" style="top: {top}px; left: {left}px; width: 50px; height: 50px;" onclick="navigateToRoom(\'{name}\')">\n'
    html += f'    <div class="room-label" style="{label_pos}">{name}</div>\n'
    html += f'</div>\n'

print(html)
