#pragma once
#include <vector>

enum class Shape { Circle, Triangle, Count };

// Graphics Objects Data: owns the shared GPU buffers (one VAO/VBO/EBO) that hold every
// template shape (unit circle, shuttle triangle, ...). Bodies never touch buffers; they ask
// GOD to draw a Shape after setting their own uniforms.
class GOD
{
    public:
        GOD() = default;
        ~GOD();                              // frees the GL objects
        GOD(const GOD&) = delete;            // GL handles must not be copied
        GOD& operator=(const GOD&) = delete;

        void addCircle(int segments);  // builds a UNIT circle and registers it
        void addTriangle();            // builds a UNIT triangle and registers it
        void upload();                 // creates VAO/VBO/EBO and sends the data; needs a GL context
        void draw(Shape s) const;      // draws one registered shape

    private:
        struct Entry { unsigned int firstIndex = 0; int indexCount = 0; };

        void addShape(Shape s, const std::vector<float>& verts,
                               const std::vector<unsigned int>& idx);

        Entry entries[(int)Shape::Count];
        unsigned int vao = 0, vbo = 0, ebo = 0;

        std::vector<float> vertexData;        // CPU copy of every shape's vertices
        std::vector<unsigned int> indexData;  // CPU copy of every shape's indices (already offset)
};
