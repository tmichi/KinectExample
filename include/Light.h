#ifndef LIGHT_H
#define LIGHT_H 1

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32__)
#include <windows.h>
#include <GL/GL.h>
#elif defined(__APPLE__)
#include <OpenGL/GL.h>
#else
#include <GL/gl.h>
#endif

class Light {
private:
        GLenum _id;
        GLfloat _position[4];
        GLfloat _ambient[4];
        GLfloat _diffuse[4];
        GLfloat _specular[4];

public :
        explicit Light (const GLenum id, const bool on = true ) : _id(id) {
                this->setPosition(0,0,1,0);
                this->setAmbient(0.1f, 0.1f, 0.1f);
                this->setDiffuse(0.6f, 0.6f, 0.6f);
                this->setSpecular(0.3f, 0.3f, 0.3f);
                
                if ( on ) this->turnOn();
                else this->turnOff();

                return;
        }

        Light& setPosition( const float x, const float y, const float z, const float w ) {
                this->_position[0] = x;
                this->_position[1] = y;
                this->_position[2] = z;
                this->_position[3] = w;
                glLightfv( _id, GL_POSITION, this->_position);
                return *this;
        };

        Light& setAmbient( const float r, const float g, const float b) {
                this->_ambient[0] = r;
                this->_ambient[1] = g;
                this->_ambient[2] = b;
                this->_ambient[3] = 1;
                glLightfv( this->_id, GL_AMBIENT, this->_ambient);
                return *this;
        }

        Light& setDiffuse( const float r, const float g, const float b) {
                this->_diffuse[0] = r;
                this->_diffuse[1] = g;
                this->_diffuse[2] = b;
                this->_diffuse[3] = 1;
                glLightfv( this->_id, GL_DIFFUSE, this->_diffuse);

                return *this;
        }

        Light& setSpecular( const float r, const float g, const float b) {
                this->_specular[0] = r;
                this->_specular[1] = g;
                this->_specular[2] = b;
                this->_specular[3] = 1;
                glLightfv( this->_id, GL_SPECULAR, this->_specular);
        
                return *this;
        }
        
        void turnOn ( void ) {
                glEnable (this->_id);
        }

        
        void turnOff ( void ) {
                glDisable (this->_id);
        }
};
#endif