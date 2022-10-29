// -*- mode: c++; -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2022, Team Spatzenhirn
 *  Copyright (c) 2014, JSK Lab
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the JSK Lab nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "riptide_rviz/OverlayTextDisplay.hpp"

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgreTexture.h>
#include <QFontDatabase>
#include <QPainter>
#include <QStaticText>
#include <QTextDocument>
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <regex>
#include <rviz_common/logging.hpp>
#include <rviz_rendering/render_system.hpp>
#include <sstream>

namespace riptide_rviz {
    OverlayTextDisplay::OverlayTextDisplay() :
        texture_width_(0),
        texture_height_(0),
        bg_color_(0, 0, 0, 0),
        fg_color_(255, 255, 255, 255.0),
        text_size_(14),
        line_width_(2),
        text_(""),
        font_(""),
        require_update_texture_(false) {
        align_bottom_property_ = new rviz_common::properties::BoolProperty( "Align Bottom", false, 
                "align text with the bottom of the overlay region", this, SLOT(updateAlignBottom()));
        invert_shadow_property_ = new rviz_common::properties::BoolProperty(
                "Invert Shadow", false, "make shadow lighter than original text", this, SLOT(updateInvertShadow()));

        hor_dist_property_ = new rviz_common::properties::IntProperty("hor_dist", 0, "horizontal distance to anchor",
                                                                      this, SLOT(updateHorizontalDistance()));
        ver_dist_property_ = new rviz_common::properties::IntProperty("ver_dist", 0, "vertical distance to anchor",
                                                                      this, SLOT(updateVerticalDistance()));

        hor_alignment_property_ = new rviz_common::properties::EnumProperty("hor_alignment", "left",
            "horizontal alignment of the overlay", this, SLOT(updateHorizontalAlignment()));
        hor_alignment_property_->addOption("left", (int)HorizontalAlignment::LEFT);
        hor_alignment_property_->addOption("center", (int)HorizontalAlignment::CENTER);
        hor_alignment_property_->addOption("right", (int)HorizontalAlignment::RIGHT);

        ver_alignment_property_ = new rviz_common::properties::EnumProperty("ver_alignment", "top",
            "vertical alignment of the overlay", this, SLOT(updateVerticalAlignment()));
        ver_alignment_property_->addOption("top", (int)VerticalAlignment::TOP);
        ver_alignment_property_->addOption("center", (int)VerticalAlignment::CENTER);
        ver_alignment_property_->addOption("bottom", (int)VerticalAlignment::BOTTOM);

        width_property_ = new rviz_common::properties::IntProperty("width", 128, "width position", this, SLOT(updateWidth()));
        width_property_->setMin(0);
        height_property_ = new rviz_common::properties::IntProperty("height", 128, "height position", this, SLOT(updateHeight()));
        height_property_->setMin(0);
        text_size_property_ = new rviz_common::properties::IntProperty("text size", 12, "text size", this, SLOT(updateTextSize()));
        text_size_property_->setMin(0);
        line_width_property_ = new rviz_common::properties::IntProperty("line width", 2, "line width", this, SLOT(updateLineWidth()));
        line_width_property_->setMin(0);

        QFontDatabase database;
        font_families_ = database.families();
        font_property_ = new rviz_common::properties::EnumProperty("font", 
                "DejaVu Sans Mono", "font", this, SLOT(updateFont()));
        for (ssize_t i = 0; i < font_families_.size(); i++) {
            font_property_->addOption(font_families_[i], (int) i);
        }
    }

    OverlayTextDisplay::~OverlayTextDisplay() {
        onDisable();
    }

    void OverlayTextDisplay::onEnable() {
        if (overlay_) {
            overlay_->show();
        }
    }

    void OverlayTextDisplay::onDisable() {
        if (overlay_) {
            overlay_->hide();
        }
    }

    // only the first time
    void OverlayTextDisplay::onInitialize() {
        rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);

        onEnable();
        updateAlignBottom();
        updateInvertShadow();
        updateHorizontalDistance();
        updateVerticalDistance();
        updateHorizontalAlignment();
        updateVerticalAlignment();
        updateWidth();
        updateHeight();
        updateTextSize();
        updateFont();
        updateLineWidth();
        require_update_texture_ = true;

        FrameProperties props = {
            "",
            300,
            100,

            12,
            0,
            0,

            QColor(0, 0, 0, 0),
            QColor(255, 255, 255, 255),

            300
        };

        setText(props, false);
    }

    void OverlayTextDisplay::update(float /*wall_dt*/, float /*ros_dt*/) {
        if (!require_update_texture_) {
            return;
        }
        if (!isEnabled()) {
            return;
        }
        if (!overlay_) {
            return;
        }

        overlay_->updateTextureSize(texture_width_, texture_height_);
        {
            ScopedPixelBuffer buffer = overlay_->getBuffer();
            QImage Hud = buffer.getQImage(*overlay_, bg_color_);
            QPainter painter(&Hud);
            painter.setRenderHint(QPainter::Antialiasing, true);
            painter.setPen(QPen(fg_color_, std::max(line_width_, 1), Qt::SolidLine));
            uint16_t w = overlay_->getTextureWidth();
            uint16_t h = overlay_->getTextureHeight();

            // font
            if (text_size_ != 0) {
                // QFont font = painter.font();
                QFont font(font_.length() > 0 ? font_.c_str() : "Liberation Sans");
                font.setPointSize(text_size_);
                font.setBold(true);
                painter.setFont(font);
            }
            if (text_.length() > 0) {

                QColor shadow_color;
                if (invert_shadow_)
                    shadow_color = Qt::white; // fg_color_.lighter();
                else
                    shadow_color = Qt::black; // fg_color_.darker();
                shadow_color.setAlpha(fg_color_.alpha());

                std::string color_wrapped_text =
                        (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>") % text_ %
                         fg_color_.red() % fg_color_.green() % fg_color_.blue() % fg_color_.alpha())
                                .str();

                // find a remove "color: XXX;" regex match to generate a proper shadow
                std::regex color_tag_re("color:.+?;");
                std::string null_char("");
                std::string formatted_text_ = std::regex_replace(text_, color_tag_re, null_char);
                std::string color_wrapped_shadow =
                        (boost::format("<span style=\"color: rgba(%2%, %3%, %4%, %5%)\">%1%</span>") % formatted_text_ %
                         shadow_color.red() % shadow_color.green() % shadow_color.blue() % shadow_color.alpha())
                                .str();

                QStaticText static_text(boost::algorithm::replace_all_copy(color_wrapped_text, "\n", "<br >").c_str());
                static_text.setTextWidth(w);

                painter.setPen(QPen(shadow_color, std::max(line_width_, 1), Qt::SolidLine));
                QStaticText static_shadow(
                        boost::algorithm::replace_all_copy(color_wrapped_shadow, "\n", "<br >").c_str());
                static_shadow.setTextWidth(w);

                if (!align_bottom_) {
                    painter.drawStaticText(1, 1, static_shadow);
                    painter.drawStaticText(0, 0, static_text);
                } else {
                    QStaticText only_wrapped_text(color_wrapped_text.c_str());
                    QFontMetrics fm(painter.fontMetrics());
                    QRect text_rect = fm.boundingRect(0, 0, w, h, Qt::TextWordWrap | Qt::AlignLeft | Qt::AlignTop,
                                                      only_wrapped_text.text().remove(QRegExp("<[^>]*>")));
                    painter.drawStaticText(1, h - text_rect.height() + 1, static_shadow);
                    painter.drawStaticText(0, h - text_rect.height(), static_text);
                }
            }
            painter.end();
        }
        overlay_->setDimensions(overlay_->getTextureWidth(), overlay_->getTextureHeight());
        require_update_texture_ = false;
    }

    void OverlayTextDisplay::setText(const FrameProperties & prop, bool show){
        if (!overlay_) {
            static int count = 0;
            std::stringstream ss;
            ss << "OverlayTextDisplayObject" << count++;
            overlay_.reset(new OverlayObject(ss.str()));
        }

        if (overlay_) {
            if (show && isEnabled()) {
                overlay_->show();
            } else {
                overlay_->hide();
            }
        }

        // store message for update method
        text_ = prop.text;

        texture_width_ = prop.texture_width_;
        texture_height_ = prop.texture_height_;
        text_size_ = prop.text_size_;
        horizontal_dist_ = prop.horizontal_dist_;
        vertical_dist_ = prop.vertical_dist_;
        
        bg_color_ = prop.bg_color_;
        fg_color_ = prop.fg_color_;
        line_width_ = prop.line_width_;
        
        if (overlay_) {
            overlay_->setPosition(horizontal_dist_, vertical_dist_, horizontal_alignment_, vertical_alignment_);
        }
        require_update_texture_ = true;
    }

    void OverlayTextDisplay::reset() {
        if (overlay_) {
            overlay_->hide();
        }
    }

    void OverlayTextDisplay::updateAlignBottom() {
        if (align_bottom_ != align_bottom_property_->getBool()) {
            require_update_texture_ = true;
        }
        align_bottom_ = align_bottom_property_->getBool();
    }

    void OverlayTextDisplay::updateInvertShadow() {
        if (invert_shadow_ != invert_shadow_property_->getBool()) {
            require_update_texture_ = true;
        }
        invert_shadow_ = invert_shadow_property_->getBool();
    }

    void OverlayTextDisplay::updateVerticalDistance() {
        vertical_dist_ = ver_dist_property_->getInt();
        require_update_texture_ = true;
    }

    void OverlayTextDisplay::updateHorizontalDistance() {
        horizontal_dist_ = hor_dist_property_->getInt();
        require_update_texture_ = true;
    }

    void OverlayTextDisplay::updateVerticalAlignment() {
        vertical_alignment_ = static_cast<VerticalAlignment>(ver_alignment_property_->getOptionInt());
        require_update_texture_ = true;
    }

    void OverlayTextDisplay::updateHorizontalAlignment() {
        horizontal_alignment_ = static_cast<HorizontalAlignment>(hor_alignment_property_->getOptionInt());
        require_update_texture_ = true;
    }

    void OverlayTextDisplay::updateWidth() {
        texture_width_ = width_property_->getInt();
        require_update_texture_ = true;
    }

    void OverlayTextDisplay::updateHeight() {
        texture_height_ = height_property_->getInt();
        require_update_texture_ = true;
    }

    void OverlayTextDisplay::updateTextSize() {
        text_size_ = text_size_property_->getInt();
        require_update_texture_ = true;
    }

    void OverlayTextDisplay::updateFont() {
        int font_index = font_property_->getOptionInt();
        if (font_index < font_families_.size()) {
            font_ = font_families_[font_index].toStdString();
        } else {
            RVIZ_COMMON_LOG_ERROR_STREAM("Unexpected error at selecting font index " << font_index);
            return;
        }

        require_update_texture_ = true;
    }

    void OverlayTextDisplay::updateLineWidth() {
        line_width_ = line_width_property_->getInt();
        require_update_texture_ = true;
    }

} // namespace rviz_2d_overlay_plugins

// Since this is a parent class this has been removed
#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(riptide_rviz::OverlayTextDisplay, rviz_common::Display)