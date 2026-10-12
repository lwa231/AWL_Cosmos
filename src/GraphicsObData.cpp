#include "GraphicsObData.h"
#include <glad/glad.h>
#include <cmath> // for cos and sin functions

const double PI = std::acos(-1.0);

GOD::~GOD() {
    // 0 means "never created"; deleting only what exists keeps this safe
    if (ebo) glDeleteBuffers(1, &ebo);
    if (vbo) glDeleteBuffers(1, &vbo);
    if (vao) glDeleteVertexArrays(1, &vao);
}

// Appends a shape's LOCAL data (indices start at 0) and records where it lives.
void GOD::addShape(Shape s, const std::vector<float>& verts,
                            const std::vector<unsigned int>& idx) {
    unsigned int baseVertex = vertexData.size() / 3; // vertices already stored
    entries[(int)s] = { (unsigned int)indexData.size(), (int)idx.size() };

    vertexData.insert(vertexData.end(), verts.begin(), verts.end());
    for (unsigned int i : idx)
        indexData.push_back(i + baseVertex);   // shift into the shared buffer
}

void GOD::addCircle(int segments) {
    std::vector<float> verts = {0.0f, 0.0f, 0.0f};      // center
    std::vector<unsigned int> idx;

    for (int i = 0; i <= segments; ++i) {               // segments+1 rim points; last repeats first
        double angle = 2.0 * PI * i / segments;
        verts.push_back((float)cos(angle));             // radius is 1: size comes from uScale
        verts.push_back((float)sin(angle));
        verts.push_back(0.0f);
    }
    for (unsigned int i = 1; i <= (unsigned int)segments; ++i) {
        idx.push_back(0);
        idx.push_back(i);
        idx.push_back(i + 1);
    }
    addShape(Shape::Circle, verts, idx);
}

void GOD::addTriangle() {
    std::vector<float> verts = {
        0.0f,  0.0f, 0.0f,
        -1.0f, 0.4f, 0.0f,   
        -1.0f, -0.4f, 0.0f  
    };
    std::vector<unsigned int> idx = { 0, 1, 2 };
    addShape(Shape::Triangle, verts, idx);
}

void GOD::upload() {
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);

    glBindVertexArray(vao);                              // VAO first: it records what follows
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, vertexData.size() * sizeof(float),
                 vertexData.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);          // bind BEFORE upload
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indexData.size() * sizeof(unsigned int),
                 indexData.data(), GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
}

void GOD::draw(Shape s) const {
    const Entry& e = entries[(int)s];
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, e.indexCount, GL_UNSIGNED_INT,
                   (void*)(e.firstIndex * sizeof(unsigned int)));  // BYTE offset into the EBO
}
