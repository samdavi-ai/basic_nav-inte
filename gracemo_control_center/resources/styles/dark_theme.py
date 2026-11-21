"""
Authoritative Minimal Monochrome Master QSS Stylesheet for GRaCEmo ViRa UARCC.
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
- Status colours used strictly for operational meaning.
"""

from .tokens import (
    COLOR_BG_MAIN,
    COLOR_WORKSPACE,
    COLOR_SURFACE,
    COLOR_SURFACE_ALT,
    COLOR_PANEL_MAIN,
    COLOR_PANEL_SECONDARY,
    COLOR_PANEL_ELEVATED,
    COLOR_CARD_BG,
    COLOR_SUBTLE_SURFACE,
    COLOR_HOVER,
    COLOR_SIDEBAR_BG,
    COLOR_SIDEBAR_SURFACE,
    COLOR_SIDEBAR_ACTIVE,
    COLOR_SIDEBAR_HOVER,
    COLOR_TEXT_SIDEBAR,
    COLOR_TEXT_SIDEBAR_MUTED,
    COLOR_BORDER,
    COLOR_BORDER_LIGHT,
    COLOR_BORDER_FOCUS,
    COLOR_BORDER_ACCENT,
    COLOR_BUTTON_PRIMARY,
    COLOR_BUTTON_PRIMARY_TEXT,
    COLOR_BUTTON_PRIMARY_HOVER,
    COLOR_BUTTON_SECONDARY,
    COLOR_BUTTON_SECONDARY_BORDER,
    COLOR_BUTTON_SECONDARY_TEXT,
    COLOR_BUTTON_SECONDARY_HOVER,
    COLOR_ACCENT_PRIMARY,
    COLOR_ACCENT_HOVER,
    COLOR_ACCENT_ACTIVE,
    COLOR_ACCENT_SECONDARY,
    COLOR_BRAND_ORANGE,
    COLOR_SUCCESS,
    COLOR_SUCCESS_BG,
    COLOR_SUCCESS_BORDER,
    COLOR_WARNING,
    COLOR_WARNING_BG,
    COLOR_WARNING_BORDER,
    COLOR_ERROR,
    COLOR_ERROR_BG,
    COLOR_ERROR_BORDER,
    COLOR_INFO,
    COLOR_INFO_BG,
    COLOR_INFO_BORDER,
    COLOR_OFFLINE,
    COLOR_OFFLINE_BG,
    COLOR_OFFLINE_BORDER,
    COLOR_TEXT_PRIMARY,
    COLOR_TEXT_SECONDARY,
    COLOR_TEXT_MUTED,
    COLOR_TEXT_LIGHT,
    FONT_FAMILY_PRIMARY,
    FONT_FAMILY_MONO,
    BORDER_RADIUS_SM,
    BORDER_RADIUS_MD,
    BORDER_RADIUS_LG,
)


def get_application_stylesheet() -> str:
    """Return the master minimal monochrome engineering QSS stylesheet."""
    return f"""
    /* ========================================================================= */
    /* 1. Global Window, Workspace & Base Typography                             */
    /* ========================================================================= */
    QMainWindow {{
        background-color: {COLOR_BG_MAIN};
        color: {COLOR_TEXT_PRIMARY};
        font-family: {FONT_FAMILY_PRIMARY};
        font-size: 13px;
    }}

    QWidget {{
        color: {COLOR_TEXT_PRIMARY};
        font-family: {FONT_FAMILY_PRIMARY};
        font-size: 13px;
    }}

    QStackedWidget {{
        background-color: {COLOR_BG_MAIN};
        border: none;
    }}

    QScrollArea {{
        background-color: transparent;
        border: none;
    }}

    /* ========================================================================= */
    /* 2. Top Application Status Bar                                             */
    /* ========================================================================= */
    QFrame#topStatusBar {{
        background-color: {COLOR_WORKSPACE};
        border-bottom: 1px solid {COLOR_BORDER};
        padding: 6px 16px;
    }}

    /* ========================================================================= */
    /* 3. Persistent Sidebar Navigation (Minimal Monochrome #F7F7F7)             */
    /* ========================================================================= */
    QFrame#sidebarPanel {{
        background-color: {COLOR_SIDEBAR_BG};
        border-right: 1px solid {COLOR_BORDER};
    }}

    QLabel.sidebarBrandTitle, QLabel[class="sidebarBrandTitle"] {{
        color: {COLOR_TEXT_PRIMARY};
        font-family: {FONT_FAMILY_PRIMARY};
        font-size: 15px;
        font-weight: 700;
        letter-spacing: 0.3px;
    }}

    QLabel.sidebarBrandSubtitle, QLabel[class="sidebarBrandSubtitle"] {{
        color: {COLOR_TEXT_SECONDARY};
        font-family: {FONT_FAMILY_MONO};
        font-size: 10px;
        font-weight: 600;
        letter-spacing: 1px;
    }}

    QLabel.sidebarCategoryHeader, QLabel[class="sidebarCategoryHeader"] {{
        color: {COLOR_TEXT_MUTED};
        font-family: {FONT_FAMILY_PRIMARY};
        font-size: 10px;
        font-weight: 600;
        letter-spacing: 0.8px;
        padding: 12px 14px 4px 14px;
        text-transform: uppercase;
    }}

    QPushButton.navButton, QPushButton[class="navButton"] {{
        background-color: transparent;
        color: {COLOR_TEXT_SECONDARY};
        border: 1px solid transparent;
        padding: 6px 10px;
        text-align: left;
        font-size: 12px;
        font-weight: 500;
        border-radius: {BORDER_RADIUS_SM}px;
        margin: 1px 8px;
    }}

    QPushButton.navButton:hover, QPushButton[class="navButton"]:hover {{
        background-color: {COLOR_HOVER};
        color: {COLOR_TEXT_PRIMARY};
        border: 1px solid transparent;
    }}

    QPushButton.navButton:checked, QPushButton[class="navButton"]:checked {{
        background-color: {COLOR_WORKSPACE};
        color: {COLOR_TEXT_PRIMARY};
        border: 1px solid {COLOR_BORDER};
        font-weight: 600;
    }}

    /* ========================================================================= */
    /* 4. Bottom Status & Notification Bar                                       */
    /* ========================================================================= */
    QFrame#bottomNotificationBar {{
        background-color: {COLOR_SUBTLE_SURFACE};
        border-top: 1px solid {COLOR_BORDER};
        padding: 5px 16px;
    }}

    /* ========================================================================= */
    /* 5. Standard Cards & Panel Workspaces (#FFFFFF with #E5E5E5 border)        */
    /* ========================================================================= */
    QFrame.cardPanel, QFrame[class="cardPanel"] {{
        background-color: {COLOR_WORKSPACE};
        border: 1px solid {COLOR_BORDER};
        border-radius: {BORDER_RADIUS_SM}px;
        padding: 14px;
    }}

    QFrame.cardPanelElevated, QFrame[class="cardPanelElevated"] {{
        background-color: {COLOR_WORKSPACE};
        border: 1px solid {COLOR_BORDER};
        border-radius: {BORDER_RADIUS_SM}px;
        padding: 14px;
    }}

    QFrame.wellPanel, QFrame[class="wellPanel"] {{
        background-color: {COLOR_SUBTLE_SURFACE};
        border: 1px solid {COLOR_BORDER};
        border-radius: {BORDER_RADIUS_SM}px;
        padding: 10px;
    }}

    /* ========================================================================= */
    /* 6. Standard Engineering Buttons                                           */
    /* ========================================================================= */
    QPushButton {{
        background-color: {COLOR_BUTTON_SECONDARY};
        color: {COLOR_BUTTON_SECONDARY_TEXT};
        border: 1px solid {COLOR_BUTTON_SECONDARY_BORDER};
        border-radius: {BORDER_RADIUS_SM}px;
        padding: 6px 14px;
        font-size: 12px;
        font-weight: 500;
    }}

    QPushButton:hover {{
        background-color: {COLOR_BUTTON_SECONDARY_HOVER};
        border-color: #BDBDBD;
    }}

    QPushButton:pressed {{
        background-color: #E5E5E5;
    }}

    QPushButton:disabled {{
        background-color: #F7F7F7;
        color: #A3A3A3;
        border-color: #E5E5E5;
    }}

    QPushButton[class="primaryButton"], QPushButton#primaryButton {{
        background-color: {COLOR_BUTTON_PRIMARY};
        color: {COLOR_BUTTON_PRIMARY_TEXT};
        border: 1px solid {COLOR_BUTTON_PRIMARY};
        border-radius: {BORDER_RADIUS_SM}px;
        padding: 6px 16px;
        font-size: 12px;
        font-weight: 600;
    }}

    QPushButton[class="primaryButton"]:hover, QPushButton#primaryButton:hover {{
        background-color: {COLOR_BUTTON_PRIMARY_HOVER};
        border-color: {COLOR_BUTTON_PRIMARY_HOVER};
    }}

    QPushButton[class="primaryButton"]:pressed, QPushButton#primaryButton:pressed {{
        background-color: #000000;
    }}

    QPushButton[class="primaryButton"]:disabled, QPushButton#primaryButton:disabled {{
        background-color: #E5E5E5;
        color: #8A8A8A;
        border-color: #E5E5E5;
    }}

    QPushButton[class="secondaryButton"], QPushButton#secondaryButton {{
        background-color: {COLOR_BUTTON_SECONDARY};
        color: {COLOR_BUTTON_SECONDARY_TEXT};
        border: 1px solid {COLOR_BUTTON_SECONDARY_BORDER};
        border-radius: {BORDER_RADIUS_SM}px;
        padding: 6px 14px;
        font-size: 12px;
        font-weight: 500;
    }}

    QPushButton[class="secondaryButton"]:hover, QPushButton#secondaryButton:hover {{
        background-color: {COLOR_BUTTON_SECONDARY_HOVER};
    }}

    QPushButton[class="dangerButton"], QPushButton#dangerButton {{
        background-color: {COLOR_ERROR};
        color: #FFFFFF;
        border: 1px solid {COLOR_ERROR};
        border-radius: {BORDER_RADIUS_SM}px;
        padding: 6px 16px;
        font-size: 12px;
        font-weight: 700;
        letter-spacing: 0.5px;
    }}

    QPushButton[class="dangerButton"]:hover, QPushButton#dangerButton:hover {{
        background-color: #B91C1C;
        border-color: #B91C1C;
    }}

    QPushButton[class="dangerButton"]:pressed, QPushButton#dangerButton:pressed {{
        background-color: #991B1B;
    }}

    /* ========================================================================= */
    /* 7. Input Fields, Combo Boxes & Terminals                                  */
    /* ========================================================================= */
    QLineEdit, QTextEdit, QPlainTextEdit {{
        background-color: {COLOR_WORKSPACE};
        border: 1px solid {COLOR_BORDER};
        border-radius: {BORDER_RADIUS_SM}px;
        color: {COLOR_TEXT_PRIMARY};
        padding: 6px 8px;
        font-family: {FONT_FAMILY_PRIMARY};
        font-size: 12px;
        selection-background-color: {COLOR_TEXT_PRIMARY};
        selection-color: #FFFFFF;
    }}

    QLineEdit:focus, QTextEdit:focus, QPlainTextEdit:focus {{
        border: 1px solid {COLOR_TEXT_PRIMARY};
    }}

    QComboBox {{
        background-color: {COLOR_WORKSPACE};
        border: 1px solid {COLOR_BORDER};
        border-radius: {BORDER_RADIUS_SM}px;
        color: {COLOR_TEXT_PRIMARY};
        padding: 5px 10px;
        font-size: 12px;
        font-weight: 500;
        min-height: 22px;
    }}

    QComboBox:hover {{
        border-color: #BDBDBD;
    }}

    QComboBox:focus {{
        border: 1px solid {COLOR_TEXT_PRIMARY};
    }}

    QComboBox::drop-down {{
        subcontrol-origin: padding;
        subcontrol-position: top right;
        width: 22px;
        border-left: 1px solid {COLOR_BORDER};
    }}

    QComboBox QAbstractItemView {{
        background-color: {COLOR_WORKSPACE};
        border: 1px solid {COLOR_BORDER};
        color: {COLOR_TEXT_PRIMARY};
        selection-background-color: {COLOR_HOVER};
        selection-color: {COLOR_TEXT_PRIMARY};
        padding: 4px;
        outline: none;
    }}

    /* ========================================================================= */
    /* 8. Data Tables & Diagnostic Grids                                         */
    /* ========================================================================= */
    QTableWidget, QTableView {{
        background-color: {COLOR_WORKSPACE};
        border: 1px solid {COLOR_BORDER};
        gridline-color: {COLOR_BORDER};
        color: {COLOR_TEXT_PRIMARY};
        selection-background-color: {COLOR_HOVER};
        selection-color: {COLOR_TEXT_PRIMARY};
        font-family: {FONT_FAMILY_PRIMARY};
        font-size: 12px;
        alternate-background-color: {COLOR_SUBTLE_SURFACE};
    }}

    QHeaderView::section {{
        background-color: {COLOR_SUBTLE_SURFACE};
        color: {COLOR_TEXT_SECONDARY};
        padding: 6px 10px;
        border: none;
        border-right: 1px solid {COLOR_BORDER};
        border-bottom: 1px solid {COLOR_BORDER};
        font-size: 11px;
        font-weight: 600;
        letter-spacing: 0.3px;
        text-transform: uppercase;
    }}

    /* ========================================================================= */
    /* 9. Progress Bars & Sliders                                                */
    /* ========================================================================= */
    QProgressBar {{
        background-color: {COLOR_SUBTLE_SURFACE};
        border: 1px solid {COLOR_BORDER};
        border-radius: {BORDER_RADIUS_SM}px;
        text-align: center;
        color: {COLOR_TEXT_SECONDARY};
        font-family: {FONT_FAMILY_MONO};
        font-size: 10px;
    }}

    QProgressBar::chunk {{
        background-color: {COLOR_TEXT_PRIMARY};
        border-radius: 3px;
    }}

    /* ========================================================================= */
    /* 10. Scrollbars                                                            */
    /* ========================================================================= */
    QScrollBar:vertical {{
        background-color: transparent;
        width: 8px;
        margin: 0px;
    }}

    QScrollBar::handle:vertical {{
        background-color: #D4D4D4;
        min-height: 24px;
        border-radius: 4px;
    }}

    QScrollBar::handle:vertical:hover {{
        background-color: #A3A3A3;
    }}

    QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical {{
        height: 0px;
    }}

    QScrollBar:horizontal {{
        background-color: transparent;
        height: 8px;
        margin: 0px;
    }}

    QScrollBar::handle:horizontal {{
        background-color: #D4D4D4;
        min-width: 24px;
        border-radius: 4px;
    }}

    QScrollBar::handle:horizontal:hover {{
        background-color: #A3A3A3;
    }}

    QScrollBar::add-line:horizontal, QScrollBar::sub-line:horizontal {{
        width: 0px;
    }}

    /* ========================================================================= */
    /* 11. Tooltips                                                              */
    /* ========================================================================= */
    QToolTip {{
        background-color: {COLOR_TEXT_PRIMARY};
        color: #FFFFFF;
        border: 1px solid {COLOR_TEXT_PRIMARY};
        padding: 4px 8px;
        font-size: 11px;
        border-radius: 3px;
    }}

    /* ========================================================================= */
    /* 12. Tabs & Tab Bars (Minimal Monochrome)                                  */
    /* ========================================================================= */
    QTabWidget::pane {{
        border: 1px solid {COLOR_BORDER};
        background-color: {COLOR_WORKSPACE};
        border-radius: {BORDER_RADIUS_SM}px;
        top: -1px;
    }}

    QTabBar::tab {{
        background-color: {COLOR_SIDEBAR_BG};
        color: {COLOR_TEXT_SECONDARY};
        border: 1px solid {COLOR_BORDER};
        border-bottom: none;
        padding: 7px 16px;
        margin-right: 4px;
        border-top-left-radius: {BORDER_RADIUS_SM}px;
        border-top-right-radius: {BORDER_RADIUS_SM}px;
        font-size: 12px;
        font-weight: 500;
    }}

    QTabBar::tab:hover {{
        background-color: {COLOR_HOVER};
        color: {COLOR_TEXT_PRIMARY};
    }}

    QTabBar::tab:selected {{
        background-color: {COLOR_WORKSPACE};
        color: {COLOR_TEXT_PRIMARY};
        border-color: {COLOR_BORDER};
        border-bottom: 1px solid {COLOR_WORKSPACE};
        font-weight: 600;
    }}

    /* ========================================================================= */
    /* 13. RViz-Inspired Engineering Workstation Menu Bar & Tool Bars            */
    /* ========================================================================= */
    QMenuBar {{
        background-color: {COLOR_WORKSPACE};
        color: {COLOR_TEXT_PRIMARY};
        border-bottom: 1px solid {COLOR_BORDER};
        font-size: 12px;
        padding: 2px 4px;
    }}

    QMenuBar::item {{
        background-color: transparent;
        padding: 4px 8px;
        margin: 1px 2px;
        border-radius: 3px;
    }}

    QMenuBar::item:selected {{
        background-color: {COLOR_HOVER};
        color: {COLOR_TEXT_PRIMARY};
    }}

    QMenu {{
        background-color: {COLOR_WORKSPACE};
        color: {COLOR_TEXT_PRIMARY};
        border: 1px solid {COLOR_BORDER};
        border-radius: 4px;
        padding: 4px 0px;
        font-size: 12px;
    }}

    QMenu::item {{
        padding: 6px 24px 6px 16px;
        margin: 0px 4px;
        border-radius: 3px;
    }}

    QMenu::item:selected {{
        background-color: {COLOR_HOVER};
        color: {COLOR_TEXT_PRIMARY};
    }}

    QMenu::separator {{
        height: 1px;
        background-color: {COLOR_BORDER};
        margin: 4px 8px;
    }}

    QToolBar {{
        background-color: {COLOR_SUBTLE_SURFACE};
        border-bottom: 1px solid {COLOR_BORDER};
        spacing: 4px;
        padding: 3px 6px;
    }}

    QToolBar::separator {{
        width: 1px;
        background-color: {COLOR_BORDER};
        margin: 4px 6px;
    }}

    QToolButton {{
        background-color: transparent;
        color: {COLOR_TEXT_PRIMARY};
        border: 1px solid transparent;
        border-radius: 3px;
        padding: 4px 8px;
        font-size: 11px;
        font-weight: 500;
    }}

    QToolButton:hover {{
        background-color: {COLOR_HOVER};
        border-color: {COLOR_BORDER};
    }}

    QToolButton:checked {{
        background-color: {COLOR_WORKSPACE};
        color: {COLOR_TEXT_PRIMARY};
        border-color: #BDBDBD;
        font-weight: 600;
    }}

    /* ========================================================================= */
    /* 14. RViz Displays Tree & Dock Panels                                      */
    /* ========================================================================= */
    QDockWidget {{
        titlebar-close-icon: none;
        titlebar-normal-icon: none;
        font-size: 12px;
        font-weight: 600;
    }}

    QDockWidget::title {{
        background-color: #EFEFEF;
        color: {COLOR_TEXT_PRIMARY};
        padding: 6px 10px;
        border-bottom: 1px solid {COLOR_BORDER};
        font-weight: 700;
        font-size: 11px;
        letter-spacing: 0.5px;
        text-transform: uppercase;
    }}

    QTreeWidget {{
        background-color: {COLOR_WORKSPACE};
        color: {COLOR_TEXT_PRIMARY};
        border: 1px solid {COLOR_BORDER};
        border-radius: 3px;
        font-size: 12px;
        outline: none;
        padding: 2px;
    }}

    QTreeWidget::item {{
        padding: 3px 4px;
        border-radius: 2px;
        min-height: 20px;
    }}

    QTreeWidget::item:hover {{
        background-color: {COLOR_HOVER};
    }}

    QTreeWidget::item:selected {{
        background-color: #E2E8F0;
        color: {COLOR_TEXT_PRIMARY};
    }}

    QTreeWidget::branch:has-children:!has-siblings:closed,
    QTreeWidget::branch:closed:has-children:has-siblings {{
        border-image: none;
        image: none;
    }}

    QSplitter::handle {{
        background-color: {COLOR_BORDER};
    }}

    QSplitter::handle:hover {{
        background-color: #A3A3A3;
    }}
    """
