
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

#pragma once

// appleseed-max-common headers.
#include "appleseed-max-common/iappleseedgeometricobject.h"

// Build options header.
#include "foundation/core/buildoptions.h"

// appleseed.renderer headers.
#include "renderer/api/scene.h"

// appleseed.foundation headers.
#include "foundation/utility/autoreleaseptr.h"

// 3ds Max headers.
#include "appleseed-max-common/_beginmaxheaders.h"
#include <iparamb2.h>
#include <maxtypes.h>
#include <ref.h>
#include <simpobj.h>
#include <strbasic.h>
#include <strclass.h>
#include "appleseed-max-common/_endmaxheaders.h"

// Forward declarations.
namespace renderer  { class Object; }
namespace renderer  { class Project; }

class FlakesMaxObject
  : public SimpleObject2
  , public IAppleseedGeometricObject
{
  public:
    static Class_ID get_class_id();

    // Constructor.
    FlakesMaxObject();

    // Animatable methods.
    void DeleteThis() override;
    void GetClassName(TSTR& s) override;
    SClass_ID SuperClassID() override;
    Class_ID ClassID() override;
    void BeginEditParams(IObjParam* ip, ULONG flags, Animatable* prev = nullptr) override;
    void EndEditParams(IObjParam* ip, ULONG flags, Animatable* next = nullptr) override;
    int NumSubs() override;
    Animatable* SubAnim(int i) override;
    TSTR SubAnimName(int i) override;
    int SubNumToRefNum(int subNum) override;
    int NumParamBlocks() override;
    IParamBlock2* GetParamBlock(int i) override;
    IParamBlock2* GetParamBlockByID(BlockID id) override;

    // ReferenceMaker methods.
    int NumRefs() override;
    RefTargetHandle GetReference(int i) override;
    void SetReference(int i, RefTargetHandle rtarg) override;
    RefResult NotifyRefChanged(
        const Interval&     changeInt,
        RefTargetHandle     hTarget,
        PartID&             partID,
        RefMessage          message,
        BOOL                propagate) override;

    // ReferenceTarget methods.
    RefTargetHandle Clone(RemapDir& remap) override;

    // BaseObject methods.
    BaseInterface* GetInterface(Interface_ID id) override;
    CreateMouseCallBack* GetCreateMouseCallBack() override;
    const MCHAR* GetObjectName() override;

    // todo: whose methods are these?
    IOResult Save(ISave* isave) override;
    IOResult Load(ILoad* iload) override;

    // SimpleObject2 methods.
    void BuildMesh(TimeValue t) override;

    // IAppleseedGeometricObject methods.
    foundation::auto_release_ptr<renderer::Object> create_object(
        renderer::Project&  project,
        renderer::Assembly& assembly,
        const char*         name,
        const TimeValue     time) override;

  private:
    IParamBlock2*   m_pblock;
};


//
// FlakesMaxObject class descriptor.
//

class FlakesMaxObjectClassDesc
  : public ClassDesc2
{
  public:
    int IsPublic() override;
    void* Create(BOOL loading) override;
    const MCHAR* ClassName() override;
    SClass_ID SuperClassID() override;
    Class_ID ClassID() override;
    const MCHAR* Category() override;
    const MCHAR* InternalName() override;
    HINSTANCE HInstance() override;
};

extern FlakesMaxObjectClassDesc g_flakesmaxobject_classdesc;
