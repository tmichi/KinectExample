#pragma once
#ifndef POINT_CLOUD_H
#define POINT_CLOUD_H 1
#include <Eigen/Dense>
#include <fstream>
#include <vector>
#include <iostream>

class PointCloud
{
private:
        PointCloud ( const PointCloud& that ) ;
        void operator = ( const PointCloud& that );
public:
        explicit PointCloud ( const int num = 0 ) {
			this->points_.reserve(num);
			this->normal_.reserve(num);
                return;
        }

        virtual ~PointCloud( void ) {
                return;
        }
        
		void resize(const int n) {
			this->points_.resize(n);
			this->normal_.resize(n);
		}

        int getNumPoints( void ) const {
                return static_cast<int>(this->points_.size());
        }

        Eigen::Vector3d getPoint( int id ) const {

                return this->points_.at( id );
        }

        Eigen::Vector3d getNormal( int id ) const {
                return this->normal_.at( id );
        }

        int addPoint( const Eigen::Vector3d& p ) {
                const int newId = static_cast<int>(this->points_.size());
                this->points_.push_back( p );
                this->normal_.push_back( Eigen::Vector3d( 1, 0, 0 ) );
                return newId;
        }
        
        void setPoint ( const int id , const Eigen::Vector3d& p ) {
                this->points_.at( id ) = p;
                return;
        }
        
        void setNormal ( const int id , const Eigen::Vector3d& n, const bool normalize = false ) {
                Eigen::Vector3d nrm = n;
                if ( normalize ) nrm.normalize();
                this->normal_.at( id ) = nrm;
                return;

        }
        void getBoundingBox( Eigen::Vector3d& bmin, Eigen::Vector3d& bmax ) {
                const int num = this->getNumPoints();
                if ( num < 1 ) return;

                bmin = this->getPoint( 0 );
                bmax = this->getPoint( 0 );

                for( int i = 0 ; i < num; ++i ) {
                        Eigen::Vector3d p = this->getPoint( i );
                        if ( p.x() < bmin.x() ) bmin.x() = p.x();
                        if ( p.y() < bmin.y() ) bmin.y() = p.y();
                        if ( p.z() < bmin.z() ) bmin.z() = p.z();
                        if ( p.x() > bmax.x() ) bmax.x() = p.x();
                        if ( p.y() > bmax.y() ) bmax.y() = p.y();
                        if ( p.z() > bmax.z() ) bmax.z() = p.z();
                }
                return;
        }

        bool readXyz ( const std::string& filename) {
                std::ifstream fin ( filename.c_str() ) ;
                if ( !fin ) {
                        std::cerr<<filename<<" read failed."<<std::endl;
                        return false;
                }
                int num;
                fin >> num;
                std::cerr<<"num. of points:"<<num<<std::endl;
                std::vector<Eigen::Vector3d> buf;
                while( !fin.eof() ) {
                        double x,y,z;
                        fin >>x>>y>>z;
                        if ( fin.eof() ) break;
                        buf.push_back(Eigen::Vector3d(x,y,z));
                }
                fin.close();

                bool normalUsed = (num * 2 == buf.size());
                if ( normalUsed ){
                        for( int i = 0 ; i < num ; ++i ) {
                                int id = this->addPoint( buf [ 2 * i] );
                                this->setNormal( id, buf[2 * i + 1]);
                        }
                }
                else {
                        Eigen::Vector3d n0(0,0,1);
                        for( int i = 0 ; i < num ; ++i ) {
                                int id = this->addPoint( buf [i] );
                                this->setNormal( id, Eigen::Vector3d(0,0,1));
                        }
                }
                return true;
        }

        bool writeXyz ( const std::string& filename, const bool saveNormal = true) {
                std::ofstream fout ( filename.c_str() ) ;
                if ( !fout ) {
                        std::cerr<<filename<<" read failed."<<std::endl;
                        return false;
                }
                const int num = this->getNumPoints();
                fout << num <<std::endl;
                for( int i = 0 ; i < num ; ++i ) {
                        if ( fout.bad() ) return false;
                        Eigen::Vector3d p = this->getPoint( i );
                        Eigen::Vector3d n = this->getNormal( i );
                        fout<<p.x()<<" "<<p.y()<<" "<<p.z();
                        if( saveNormal ) fout<<" "<<n.x()<<" "<<n.y()<<" "<<n.z();
                        fout<<std::endl;
                }
                fout.close();
                return true;
        }
        bool readPnt ( const std::string& filename ) {
                std::ifstream fin ( filename.c_str() ) ;
                if ( !fin ) {
                        std::cerr<<filename<<" read failed."<<std::endl;
                        return false;
                }
                while ( 1 ) {
                        if ( fin.bad() ) return false;
                        double x, y, z;
                        fin >>x>>y>>z;
                        if ( fin.eof() ) break;
			this->addPoint( Eigen::Vector3d( x,y,z ) );
                }
                std::cerr<<this->getNumPoints()<<" points read."<<std::endl;
                fin.close();
                return true;
        }
        bool writePnt ( const std::string& filename ) {
                std::ofstream fout ( filename.c_str() ) ;
                if ( !fout ) {
                        std::cerr<<filename<<" read failed."<<std::endl;
                        return false;
                }
                const int num = this->getNumPoints();
                for( int i = 0 ; i < num ; ++i ) {
                        if ( fout.bad() ) return false;
                        Eigen::Vector3d p = this->getPoint( i );
                        fout<<p.x()<<" "<<p.y()<<" "<<p.z()<<" "<<std::endl;
                }
                fout.close();
                return true;
        }
private:
        std::vector<Eigen::Vector3d> points_;
        std::vector<Eigen::Vector3d> normal_;
};

#endif// POINT_CLOUD
