// Copyright 2026 FML
// Licensed under the Apache License, Version 2.0.
#ifndef LOCALIZATION_VALUE_OVERLAY_HPP_
#define LOCALIZATION_VALUE_OVERLAY_HPP_

#include <QLinearGradient>
#include <QPainter>

namespace rviz_plugins
{
inline void paint_value_overlay(
  QPainter & painter, const QString & label, double value, bool fresh, const QColor & color,
  double minimum, double maximum, const QColor & min_color, const QColor & max_color)
{
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(0, 0, 0, 150));
  painter.drawRoundedRect(QRectF(0, 0, 200, 112), 16, 16);
  painter.setPen(QColor(174, 174, 174));
  painter.setFont(QFont("Quicksand", 11, QFont::DemiBold));
  painter.drawText(QRect(16, 8, 168, 22), Qt::AlignLeft | Qt::AlignVCenter, label);
  painter.setPen(fresh ? color : QColor(130, 130, 130));
  painter.setFont(QFont("Quicksand", 27, QFont::DemiBold));
  painter.drawText(
    QRect(16, 30, 168, 42), Qt::AlignLeft | Qt::AlignVCenter,
    fresh ? QString::number(value, 'f', 2) : "—");
  QLinearGradient gradient(16, 0, 184, 0);
  gradient.setColorAt(0, min_color);
  gradient.setColorAt(1, max_color);
  painter.setPen(Qt::NoPen);
  painter.setBrush(gradient);
  painter.drawRoundedRect(QRectF(16, 78, 168, 4), 2, 2);
  painter.setPen(QColor(174, 174, 174));
  painter.setFont(QFont("Quicksand", 8));
  painter.drawText(QRect(16, 87, 80, 17), Qt::AlignLeft, QString::number(minimum, 'f', 2));
  painter.drawText(QRect(104, 87, 80, 17), Qt::AlignRight, QString::number(maximum, 'f', 2));
  if (!fresh) painter.drawText(QRect(55, 87, 90, 17), Qt::AlignCenter, "NO DATA");
}
}  // namespace rviz_plugins
#endif  // LOCALIZATION_VALUE_OVERLAY_HPP_
