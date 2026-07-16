#ifndef FILTERSETTINGS_H
#define FILTERSETTINGS_H

#include <QObject>
#include <QtQml/qqmlregistration.h>

// 비디오 필터 파라미터 상태. QML ShaderEffect의 uniform 입력으로 사용된다.
class FilterSettings : public QObject
{
    Q_OBJECT
    QML_ELEMENT
    Q_PROPERTY(int brightness READ brightness WRITE setBrightness NOTIFY changed)
    Q_PROPERTY(int contrast READ contrast WRITE setContrast NOTIFY changed)
    Q_PROPERTY(int saturation READ saturation WRITE setSaturation NOTIFY changed)
    Q_PROPERTY(int sharpen READ sharpen WRITE setSharpen NOTIFY changed)
    Q_PROPERTY(bool grayscale READ grayscale WRITE setGrayscale NOTIFY changed)

public:
    explicit FilterSettings(QObject *parent = nullptr);

    int brightness() const { return m_brightness; }   // -100 ~ 100
    int contrast() const { return m_contrast; }       // -100 ~ 100
    int saturation() const { return m_saturation; }   // -100 ~ 100
    int sharpen() const { return m_sharpen; }         //    0 ~ 100
    bool grayscale() const { return m_grayscale; }

    void setBrightness(int v);
    void setContrast(int v);
    void setSaturation(int v);
    void setSharpen(int v);
    void setGrayscale(bool v);

    Q_INVOKABLE void reset();

signals:
    void changed();

private:
    int m_brightness = 0;
    int m_contrast = 0;
    int m_saturation = 0;
    int m_sharpen = 0;
    bool m_grayscale = false;
};

#endif // FILTERSETTINGS_H
