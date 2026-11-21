"""
Clean, Minimal Monochrome Engineering Design Tokens for GRaCEmo ViRa UARCC.
Specification:
- Main background: #FFFFFF
- Secondary background: #F7F7F7
- Primary text: #171717
- Secondary text: #666666
- Muted text: #8A8A8A
- Borders: #E5E5E5
- Hover background: #F0F0F0
- Primary button: #171717 (text: #FFFFFF)
- Secondary button: #FFFFFF (border: #D4D4D4)
- Status colours:
  - READY: #16A34A (bg: #F0FDF4, border: #DCFCE7)
  - WARNING: #D97706 (bg: #FFFBEB, border: #FEF3C7)
  - ERROR: #DC2626 (bg: #FEF2F2, border: #FEE2E2)
  - OFFLINE: #737373 (bg: #F5F5F5, border: #E5E5E5)
  - ACTIVE: #2563EB (bg: #EFF6FF, border: #DBEAFE)
"""

# Foundational Surfaces & Workspaces (Monochrome Engineering)
COLOR_BG_MAIN = "#FFFFFF"           # Main application canvas background
COLOR_BG_DARK = "#FFFFFF"           # Backward compatibility alias
COLOR_WORKSPACE = "#FFFFFF"         # Primary card and content workspace
COLOR_SURFACE = "#FFFFFF"           # Primary content panels
COLOR_PANEL_MAIN = "#FFFFFF"        # Primary content cards
COLOR_SURFACE_ALT = "#F7F7F7"       # Secondary panels / wells / input fields
COLOR_PANEL_SECONDARY = "#F7F7F7"   # Secondary cards and embedded containers
COLOR_PANEL_ELEVATED = "#FFFFFF"    # Elevated cards
COLOR_CARD_BG = "#FFFFFF"           # Standard card background
COLOR_SUBTLE_SURFACE = "#F7F7F7"    # Subtle table headers / section wells
COLOR_HOVER = "#F0F0F0"             # Interactive hover surface

# Sidebar Navigation Surfaces (Minimal Monochrome #F7F7F7)
COLOR_SIDEBAR_BG = "#F7F7F7"        # Clean secondary background sidebar
COLOR_SIDEBAR_SURFACE = "#F7F7F7"   # Secondary sidebar surface
COLOR_SIDEBAR_ACTIVE = "#FFFFFF"    # Active item background
COLOR_SIDEBAR_HOVER = "#F0F0F0"     # Hover item background
COLOR_TEXT_SIDEBAR = "#171717"      # Active sidebar text
COLOR_TEXT_SIDEBAR_MUTED = "#666666"# Inactive sidebar text

# Typography & Content Contrast
COLOR_TEXT_PRIMARY = "#171717"      # Primary text (deep dark)
COLOR_TEXT_SECONDARY = "#666666"    # Secondary text
COLOR_TEXT_MUTED = "#8A8A8A"        # Muted text (metadata, units)
COLOR_TEXT_DARK = "#171717"         # Dark text
COLOR_TEXT_LIGHT = "#FFFFFF"        # Pure white for buttons and badges

# Borders & Hairlines
COLOR_BORDER = "#E5E5E5"            # Standard crisp border
COLOR_BORDER_LIGHT = "#E5E5E5"      # Inner divider
COLOR_BORDER_FOCUS = "#171717"      # Active focus ring
COLOR_BORDER_ACCENT = "#D4D4D4"     # Accent border

# Primary & Secondary Button Tokens
COLOR_BUTTON_PRIMARY = "#171717"
COLOR_BUTTON_PRIMARY_TEXT = "#FFFFFF"
COLOR_BUTTON_PRIMARY_HOVER = "#2E2E2E"
COLOR_BUTTON_SECONDARY = "#FFFFFF"
COLOR_BUTTON_SECONDARY_BORDER = "#D4D4D4"
COLOR_BUTTON_SECONDARY_TEXT = "#171717"
COLOR_BUTTON_SECONDARY_HOVER = "#F0F0F0"

# Minimal Engineering Accents (Restrained monochrome & functional blue)
COLOR_ACCENT_PRIMARY = "#171717"    # Primary monochrome accent
COLOR_ACCENT_HOVER = "#2E2E2E"
COLOR_ACCENT_ACTIVE = "#000000"
COLOR_ACCENT_SECONDARY = "#2563EB"  # Functional restrained blue
COLOR_ACCENT_CYAN = "#2563EB"       # Functional blue alias
COLOR_ACCENT_CYAN_HOVER = "#1D4ED8"
COLOR_ACCENT_TEAL = "#2563EB"
COLOR_ACCENT_WHITE = "#FFFFFF"

# Hardware Safety Brand Accent
COLOR_BRAND_ORANGE = "#DC2626"      # Safety indicator
COLOR_ACCENT_ORANGE = "#DC2626"
COLOR_ACCENT_ORANGE_HOVER = "#B91C1C"
COLOR_ACCENT_ORANGE_DIM = "#991B1B"

# Status Colours & High-Legibility Pill Tokens
COLOR_SUCCESS = "#16A34A"           # READY / ONLINE / PASS (Restrained Green)
COLOR_SUCCESS_BG = "#F0FDF4"
COLOR_SUCCESS_BORDER = "#DCFCE7"

COLOR_WARNING = "#D97706"           # WARNING / INITIALIZING / STALE (Amber)
COLOR_WARNING_BG = "#FFFBEB"
COLOR_WARNING_BORDER = "#FEF3C7"

COLOR_ERROR = "#DC2626"             # CRITICAL / ERROR / E-STOP (Red)
COLOR_ERROR_BG = "#FEF2F2"
COLOR_ERROR_BORDER = "#FEE2E2"

COLOR_INFO = "#2563EB"              # ACTIVE / INFO (Restrained Blue)
COLOR_INFO_BG = "#EFF6FF"
COLOR_INFO_BORDER = "#DBEAFE"

COLOR_OFFLINE = "#737373"           # NOT CONNECTED / OFFLINE (Neutral Grey)
COLOR_OFFLINE_BG = "#F5F5F5"
COLOR_OFFLINE_BORDER = "#E5E5E5"

COLOR_ACTIVE = "#2563EB"
COLOR_ACTIVE_BG = "#EFF6FF"
COLOR_ACTIVE_BORDER = "#DBEAFE"

# Typography Families
FONT_FAMILY_PRIMARY = "Inter, -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, Helvetica, Arial, sans-serif"
FONT_FAMILY_MONO = "'JetBrains Mono', 'IBM Plex Mono', 'Fira Code', 'DejaVu Sans Mono', Consolas, monospace"

# Metrics & Geometry (Crisp, Restrained Radii)
BORDER_RADIUS_SM = 4
BORDER_RADIUS_MD = 6
BORDER_RADIUS_LG = 8

# RViz Engineering Workstation Tokens
COLOR_RVIZ_VIEWPORT_BG = "#1E1E1E"    # Neutral dark-grey 3D canvas (matching RViz / CAD)
COLOR_RVIZ_DOCK_BG = "#F7F7F7"        # Crisp engineering panel background
COLOR_RVIZ_GRID = "#404040"           # Subtle 3D grid line
COLOR_RVIZ_GRID_MAJOR = "#606060"     # Major 1m grid line
COLOR_RVIZ_AXIS_X = "#E02424"         # Standard ROS coordinate X axis (Red)
COLOR_RVIZ_AXIS_Y = "#16A34A"         # Standard ROS coordinate Y axis (Green)
COLOR_RVIZ_AXIS_Z = "#2563EB"         # Standard ROS coordinate Z axis (Blue)
COLOR_RVIZ_SELECTION = "#F59E0B"      # Target selection highlight (Amber)

