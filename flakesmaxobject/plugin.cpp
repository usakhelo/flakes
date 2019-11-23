
//
// This source file is part of appleseed.
// Visit https://appleseedhq.net/ for additional information and resources.
//
// This software is released under the MIT license.
//
// Copyright (c) 2018-2019 Francois Beaune, The appleseedhq Organization
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

// Project headers.
#include "flakesmaxobject.h"

// 3ds Max headers.
#include "appleseed-max-common/_beginmaxheaders.h"
#include <assert1.h>
#include <plugapi.h>
#include "appleseed-max-common/_endmaxheaders.h"

extern "C"
{
    __declspec(dllexport)
    const wchar_t* LibDescription()
    {
        return L"appleseed Flakes";
    }

    __declspec(dllexport)
    int LibNumberClasses()
    {
        return 1;
    }

    __declspec(dllexport)
    ClassDesc2* LibClassDesc(int i)
    {
        switch (i)
        {
          case 0: return &g_flakesmaxobject_classdesc;

          // Make sure to update LibNumberClasses() if you add classes here.

          default:
            DbgAssert(false);
            return nullptr;
        }
    }

    __declspec(dllexport)
    ULONG LibVersion()
    {
        return VERSION_3DSMAX;
    }

    __declspec(dllexport)
    int LibInitialize()
    {
        return TRUE;
    }

    __declspec(dllexport)
    ULONG CanAutoDefer()
    {
        return FALSE;
    }
}
