#include "filtersettings.h"

#include <algorithm>

FilterSettings::FilterSettings(QObject *parent)
    : QObject(parent)
{
}

void FilterSettings::setBrightness(int v)
{
    v = std::clamp(v, -100, 100);
    if (m_brightness == v)
        return;
    m_brightness = v;
    emit changed();
}

void FilterSettings::setContrast(int v)
{
    v = std::clamp(v, -100, 100);
    if (m_contrast == v)
        return;
    m_contrast = v;
    emit changed();
}

void FilterSettings::setSaturation(int v)
{
    v = std::clamp(v, -100, 100);
    if (m_saturation == v)
        return;
    m_saturation = v;
    emit changed();
}

void FilterSettings::setSharpen(int v)
{
    v = std::clamp(v, 0, 100);
    if (m_sharpen == v)
        return;
    m_sharpen = v;
    emit changed();
}

void FilterSettings::setGrayscale(bool v)
{
    if (m_grayscale == v)
        return;
    m_grayscale = v;
    emit changed();
}

void FilterSettings::reset()
{
    const bool dirty = m_brightness || m_contrast || m_saturation || m_sharpen || m_grayscale;
    m_brightness = 0;
    m_contrast = 0;
    m_saturation = 0;
    m_sharpen = 0;
    m_grayscale = false;
    if (dirty)
        emit changed();
}
