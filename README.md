# QOAED

## Brief / Resumen
This library implements a simple version of Quadtrees and Octrees (in both of their forms,
Point and Region), this library was created for educational purposes, please don't try to
use it in production systems.

Esta libreria implementa una versión simple de Quadtrees y Octrees (en sus dos formas,
Point y Region), esta libreria fue creada con fines educativos, por favor no intente
usarla en sistemas de producción.

## Dependencies / Dependencias
- PCL Library (for runtime)
- VTK (for runtime)
- CMake (for build)

## Compiling demos / Compilando demos
```
$ git clone https://github.com/lans98/Octree-CPP
$ cd Octree-CPP
$ mkdir build 
$ cd build 
$ cmake -G "Unix Makefiles" ..
$ cmake --build .
```

## Using demos / Usando demos
The demos are in `bin` folder inside `build` after following the above step.

Los demos estan en la carpeta `bin` dentro del directorio `build` después de seguir el
paso de arriba. 

## Including in other projects / Incluyendo en otros projectos
To use this implementations in other projects just make sure that you set the `include`
directory as an include path when you are compiling your project, or simply copy `include`
files in your project folder.

Si quieres usar estas implementaciones en otros proyectos asegúrate de que configures la
carpeta `include` como una ruta de inclusión cuando estes compilando tu proyecto, o
simplemente copia los archivos en `include` en la carpeta de tu proyecto.

## Contributors / Contribuidores
- [lans98](https://github.com/lans98)
- [k-st](https://github.com/k-st)
- [MBlev](https://github.com/MBlev)

## Further read / Lectura profunda
- [Slides on Google](https://docs.google.com/presentation/d/1YjIul8P9xd02vFztmL2MrrHZWiIlwffryJpgWpFVpxM/edit?usp=sharing)
- [Video on Youtube](https://www.youtube.com/watch?v=ITGjyHDG5yQ)
