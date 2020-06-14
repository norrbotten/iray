#pragma once

#include <iostream>
#include <vector>

#include <lodepng.h>

namespace iray {

    struct color {
        float r, g, b;

        color operator/(float rhs) {
            float inv = 1.f / rhs;
            return color{r * inv, g * inv, b * inv};
        }
    };

    class film {
        std::vector<color> data;
        std::vector<int>   samples;

        int w, h;

    public:
        film(int width, int height)
            : w(width)
            , h(height) {

            data.reserve(this->w * this->h);
            samples.reserve(this->w * this->h);
        }

        void splat(int x, int y, color col) {
            data[x + y * this->w] = col;
            samples[x + y * this->w] += 1;
        }

        color get(int x, int y) {
            if (samples[x + y * this->w] == 0)
                return color{0, 0, 0};

            return data[x + y * this->w] / float(samples[x + y * this->w]);
        }

        bool save_png(const char* filename) {
            std::vector<unsigned char> image;
            image.reserve(4 * this->w * this->h * sizeof(unsigned char));

            for (int y = 0; y < this->h; y++)
                for (int x = 0; x < this->w; x++) {
                    auto color = this->get(x, y);

                    image.push_back((unsigned char)(color.r * 255.f));
                    image.push_back((unsigned char)(color.g * 255.f));
                    image.push_back((unsigned char)(color.b * 255.f));
                    image.push_back((unsigned char)255);
                }

            std::vector<unsigned char> png;

            auto error = lodepng::encode(png, image, this->w, this->h);

            if (error) {
                std::cerr << lodepng_error_text(error);
                return true;
            }

            lodepng::save_file(png, filename);

            return false;
        }
    };

} // namespace iray
