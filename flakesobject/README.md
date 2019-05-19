## Build Instructions on Windows

Assuming that:

- You are using Visual Studio 2015
- appleseed is in `C:\appleseed\`
- windows-deps is in `C:\windows-deps\`
- Boost is in `C:\boost_1_69_0\`

In `flakesobject\`:

    mkdir build
    cd build
    cmake ^
    -G "Visual Studio 14 2015 Win64" ^
    -DAPPLESEED_INCLUDE_DIR=C:\appleseed\src\appleseed ^
    -DAPPLESEED_LIBRARY=C:\appleseed\sandbox\lib\vc140\Ship\appleseed.lib ^
    -DBOOST_ROOT=C:\boost_1_69_0 ^
    -DIMATH_INCLUDE_DIR=C:\windows-deps\stage\vc140\ilmbase-release\include ^
    -DIMATH_MATH_LIBRARY=C:\windows-deps\stage\vc140\ilmbase-release\lib\Imath-2_2.lib ^
    -DIMATH_IEX_LIBRARY=C:\windows-deps\stage\vc140\ilmbase-release\lib\Iex-2_2.lib ^
    -DIMATH_HALF_LIBRARY=C:\windows-deps\stage\vc140\ilmbase-release\lib\Half.lib ^
    -DOPENEXR_INCLUDE_DIR=C:\windows-deps\stage\vc140\openexr-release\include ^
    -DOPENEXR_IMF_LIBRARY=C:\windows-deps\stage\vc140\openexr-release\lib\IlmImf-2_2.lib ^
    -DOPENEXR_THREADS_LIBRARY=C:\windows-deps\stage\vc140\ilmbase-release\lib\IlmThread-2_2.lib ^
    -DAPPLESEED_DEPS_STAGE_DIR=C:\windows-deps\stage\vc140 ^
    ..
