"""Code editor widget with line numbers, syntax highlighting, and error/execution markers."""

from PyQt5.QtCore import QRect, QRegularExpression, QSize, Qt
from PyQt5.QtGui import (
    QColor,
    QFont,
    QPainter,
    QSyntaxHighlighter,
    QTextCharFormat,
    QTextCursor,
    QTextFormat,
)
from PyQt5.QtWidgets import QPlainTextEdit, QTextEdit, QWidget


class Lite6ScriptHighlighter(QSyntaxHighlighter):
    """Syntax highlighter for RobotScript motion programming DSL (light theme)."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self._rules = []

        def fmt(color_hex, bold=False, italic=False):
            f = QTextCharFormat()
            f.setForeground(QColor(color_hex))
            if bold:
                f.setFontWeight(QFont.Bold)
            if italic:
                f.setFontItalic(True)
            return f

        # Declarations (joint, pose)
        decl_fmt = fmt("#7C3AED", bold=True)
        for kw in ("joint", "pose"):
            self._rules.append((QRegularExpression(rf"\b{kw}\b"), decl_fmt))

        # Motion & structured control statements
        cmd_fmt = fmt("#0369A1", bold=True)
        for kw in (
            "MoveJ",
            "MoveP",
            "MoveL",
            "MoveC",
            "MovePVA",
            "ImpedanceCartesian",
            "Gripper",
            "Wait",
            "Pose",
            "point",
        ):
            self._rules.append((QRegularExpression(rf"\b{kw}\b"), cmd_fmt))

        # Parameter names
        param_fmt = fmt("#9A3412")
        for kw in (
            "vel",
            "acc",
            "jerk",
            "blend",
            "via",
            "target",
            "center",
            "axis",
            "angle",
            "target_ori",
            "space",
            "p",
            "v",
            "a",
            "dt",
            "kt",
            "kr",
            "dr",
            "null_damping",
            "transition_time",
            "action",
            "wait",
            "timeout",
            "duration",
            "xyz",
            "rpy",
            "quat",
        ):
            self._rules.append((QRegularExpression(rf"\b{kw}\b"), param_fmt))

        # Enum constants, units, and booleans
        enum_fmt = fmt("#0F766E", bold=True)
        for kw in (
            "JOINT",
            "CARTESIAN",
            "OPEN",
            "CLOSE",
            "STOP",
            "true",
            "false",
            "rad",
            "deg",
        ):
            self._rules.append((QRegularExpression(rf"\b{kw}\b"), enum_fmt))

        # Numeric literals
        num_fmt = fmt("#047857")
        self._rules.append(
            (QRegularExpression(r"(?<![A-Za-z0-9_])[-+]?(?:\d+\.?\d*|\.\d+)(?:[eE][-+]?\d+)?"), num_fmt)
        )

        # Comments
        comment_fmt = fmt("#64748B", italic=True)
        self._comment_rule = (QRegularExpression(r"#[^\n]*"), comment_fmt)

    def highlightBlock(self, text):
        for pattern, char_fmt in self._rules:
            it = pattern.globalMatch(text)
            while it.hasNext():
                m = it.next()
                self.setFormat(m.capturedStart(), m.capturedLength(), char_fmt)
        # Apply comments last so tokens inside comments stay muted
        it = self._comment_rule[0].globalMatch(text)
        while it.hasNext():
            m = it.next()
            self.setFormat(m.capturedStart(), m.capturedLength(), self._comment_rule[1])


class _LineNumberArea(QWidget):
    """Gutter widget painting line numbers alongside CodeEditor."""

    def __init__(self, editor):
        super().__init__(editor)
        self._editor = editor

    def sizeHint(self):
        return QSize(self._editor.line_number_area_width(), 0)

    def paintEvent(self, event):
        self._editor.paint_line_number_area(event)


class CodeEditor(QPlainTextEdit):
    """Industrial trajectory script editor with gutter diagnostics and live line tracking."""

    def __init__(self, parent=None):
        super().__init__(parent)
        font = QFont("Noto Sans Mono", 10)
        font.setStyleHint(QFont.TypeWriter)
        self.setFont(font)
        self.setTabStopDistance(self.fontMetrics().horizontalAdvance(" ") * 2)
        self.setLineWrapMode(QPlainTextEdit.NoWrap)
        self.setStyleSheet(
            "QPlainTextEdit {"
            "  background-color: #FFFFFF;"
            "  color: #0F172A;"
            "  border: 1px solid #DDE3EC;"
            "  border-radius: 6px;"
            "  selection-background-color: #DBEAFE;"
            "  selection-color: #1E3A8A;"
            "}"
        )

        self._highlighter = Lite6ScriptHighlighter(self.document())
        self._line_area = _LineNumberArea(self)
        self._error_lines = set()
        self._warning_lines = set()
        self._active_line = 0

        self.blockCountChanged.connect(self._update_line_number_area_width)
        self.updateRequest.connect(self._update_line_number_area)
        self.cursorPositionChanged.connect(self._refresh_extra_selections)

        self._update_line_number_area_width(0)
        self._refresh_extra_selections()

    def line_number_area_width(self):
        digits = max(3, len(str(max(1, self.blockCount()))))
        return 26 + self.fontMetrics().horizontalAdvance("9") * digits

    def _update_line_number_area_width(self, _):
        self.setViewportMargins(self.line_number_area_width(), 0, 0, 0)

    def _update_line_number_area(self, rect, dy):
        if dy:
            self._line_area.scroll(0, dy)
        else:
            self._line_area.update(0, rect.y(), self._line_area.width(), rect.height())
        if rect.contains(self.viewport().rect()):
            self._update_line_number_area_width(0)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        cr = self.contentsRect()
        self._line_area.setGeometry(
            QRect(cr.left(), cr.top(), self.line_number_area_width(), cr.height())
        )

    def paint_line_number_area(self, event):
        painter = QPainter(self._line_area)
        painter.fillRect(event.rect(), QColor("#F8FAFC"))

        # Clean 1px separator between gutter and white editor surface
        sep_x = self._line_area.width() - 1
        painter.setPen(QColor("#E2E8F0"))
        painter.drawLine(sep_x, event.rect().top(), sep_x, event.rect().bottom())

        block = self.firstVisibleBlock()
        block_number = block.blockNumber()
        top = round(self.blockBoundingGeometry(block).translated(self.contentOffset()).top())
        bottom = top + round(self.blockBoundingRect(block).height())
        line_h = self.fontMetrics().height()

        while block.isValid() and top <= event.rect().bottom():
            if block.isVisible() and bottom >= event.rect().top():
                ln = block_number + 1
                if ln in self._error_lines:
                    painter.fillRect(2, top + 3, 4, max(4, line_h - 6), QColor("#DC2626"))
                elif ln in self._warning_lines:
                    painter.fillRect(2, top + 3, 4, max(4, line_h - 6), QColor("#D97706"))
                if ln == self._active_line:
                    painter.fillRect(8, top + 3, 4, max(4, line_h - 6), QColor("#0284C7"))

                num_color = (
                    QColor("#DC2626")
                    if ln in self._error_lines
                    else QColor("#0284C7")
                    if ln == self._active_line
                    else QColor("#94A3B8")
                )
                painter.setPen(num_color)
                painter.drawText(
                    0,
                    top,
                    self._line_area.width() - 8,
                    line_h,
                    Qt.AlignRight | Qt.AlignVCenter,
                    str(ln),
                )
            block = block.next()
            top = bottom
            bottom = top + round(self.blockBoundingRect(block).height())
            block_number += 1

    def set_diagnostic_markers(self, error_lines, warning_lines=None):
        self._error_lines = set(error_lines or ())
        self._warning_lines = set(warning_lines or ())
        self._refresh_extra_selections()
        self._line_area.update()

    def set_active_line(self, line_number):
        line_number = int(line_number or 0)
        if line_number != self._active_line:
            self._active_line = line_number
            self._refresh_extra_selections()
            self._line_area.update()

    def jump_to_line(self, line_number, column_number=1):
        line_idx = max(0, min(self.blockCount() - 1, int(line_number) - 1))
        block = self.document().findBlockByNumber(line_idx)
        if not block.isValid():
            return
        col_offset = max(0, min(len(block.text()), int(column_number) - 1))
        cursor = QTextCursor(block)
        cursor.movePosition(QTextCursor.Right, QTextCursor.MoveAnchor, col_offset)
        self.setTextCursor(cursor)
        self.centerCursor()
        self.setFocus()

    def insert_snippet_at_cursor(self, text):
        cursor = self.textCursor()
        cursor.insertText(text)
        self.setTextCursor(cursor)
        self.setFocus()

    def _line_selection(self, line_num, bg_color):
        block = self.document().findBlockByNumber(line_num - 1)
        if not block.isValid():
            return None
        sel = QTextEdit.ExtraSelection()
        sel.format.setBackground(bg_color)
        sel.format.setProperty(QTextFormat.FullWidthSelection, True)
        sel.cursor = QTextCursor(block)
        sel.cursor.clearSelection()
        return sel

    def _refresh_extra_selections(self):
        selections = []
        if not self.isReadOnly():
            sel = QTextEdit.ExtraSelection()
            sel.format.setBackground(QColor("#F1F5F9"))
            sel.format.setProperty(QTextFormat.FullWidthSelection, True)
            sel.cursor = self.textCursor()
            sel.cursor.clearSelection()
            selections.append(sel)

        for ln in sorted(self._warning_lines):
            s = self._line_selection(ln, QColor(245, 158, 11, 30))
            if s is not None:
                selections.append(s)

        for ln in sorted(self._error_lines):
            s = self._line_selection(ln, QColor(239, 68, 68, 34))
            if s is not None:
                selections.append(s)

        if self._active_line > 0:
            s = self._line_selection(self._active_line, QColor(14, 165, 233, 36))
            if s is not None:
                selections.append(s)

        self.setExtraSelections(selections)
