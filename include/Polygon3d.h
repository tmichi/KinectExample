#ifndef POLYGON3D_H
#define POLYGON3D_H 1
#include <Eigen/Dense>
#include <vector>

class Polygon3d {
private:
        std::vector<Eigen::Vector3d> _points;
        Eigen::Vector3d _normal;
public:
        Polygon3d ( void  ) {
                return;
        };

        Polygon3d ( const Polygon3d& that ) {
                this->_points.clear();
                this->_points.insert( this->_points.end(), that._points.begin(), that._points.end());
                this->_normal = that._normal;
        }

        Polygon3d& operator = ( const Polygon3d& that ) {
                this->_points.clear();
                this->_points.insert( this->_points.end(), that._points.begin(), that._points.end());
                this->_normal = that._normal;
                return *this;
        }

        int getNumPoints ( void ) const {
                return static_cast<int>( this->_points.size());
        }
        void setNormal( const Eigen::Vector3d& n ) {
                this->_normal = n;
        }
        void setPoint (const Eigen::Vector3d& p ) {
                this->_points.push_back(p);
        }

        Eigen::Vector3d getNormal ( void ) const {
                return this->_normal;
        }

        Eigen::Vector3d getPoint( const int id ) const {
                return this->_points[id];
        }

};
#endif //POLYGON3D_H