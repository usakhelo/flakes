
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

// Interface header.
#include "flakesmaxobject.h"

// Project headers.
#include "datachunks.h"
#include "resource.h"
#include "main.h"

// appleseed-max-common headers.
#include "appleseed-max-common/utilities.h"

// Build options header.
#include "foundation/core/buildoptions.h"

// appleseed.renderer headers.
#include "renderer/api/log.h"
#include "renderer/api/object.h"
#include "renderer/api/project.h"
#include "renderer/api/utility.h"

// appleseed.foundation headers.
#include "foundation/platform/windows.h"
#include "foundation/utility/countof.h"

// 3ds Max headers.
#include "appleseed-max-common/_beginmaxheaders.h"
#include <assert1.h>
#include <paramtype.h>
#include "appleseed-max-common/_endmaxheaders.h"

// Boost headers.
#include "boost/filesystem.hpp"

// Standard headers.
#include <string>

// Windows headers.
#include <Shlwapi.h>

namespace asf = foundation;
namespace asr = renderer;
namespace bf = boost::filesystem;

namespace
{
    const wchar_t* FlakesMaxObjectFriendlyClassName = L"Flakes";
}

FlakesMaxObjectClassDesc g_flakesmaxobject_classdesc;


//
// FlakesMaxObject class implementation.
//

namespace
{
    enum { ParamBlockIdFlakesMaxObject };
    enum { ParamBlockRefFlakesMaxObject };

    enum ParamId
    {
        // Changing these value WILL break compatibility.
        ParamIdBaseObject           = 0,
        ParamIdMode                 = 1,
        ParamIdVoxelSize            = 2,
        ParamIdFlakeSize            = 3,
        ParamIdFlakeSizeJitter      = 4,
        ParamIdFlakeCenterJitter    = 5,
        ParamIdFlakesPerVoxel       = 6
    };

    ParamBlockDesc2 g_block_desc(
        // --- Required arguments ---
        ParamBlockIdFlakesMaxObject,                // parameter block's ID
        L"flakesMaxObjectParams",                   // internal parameter block's name
        0,                                          // ID of the localized name string
        &g_flakesmaxobject_classdesc,               // class descriptor
        P_AUTO_CONSTRUCT + P_AUTO_UI,               // block flags

        // --- P_AUTO_CONSTRUCT arguments ---
        ParamBlockRefFlakesMaxObject,               // parameter block's reference number

        // --- P_AUTO_UI arguments ---
        IDD_FORMVIEW_PARAMS,                        // ID of the dialog template
        IDS_FORMVIEW_PARAMS_TITLE,                  // ID of the dialog's title string
        0,                                          // IParamMap2 creation/deletion flag mask
        0,                                          // rollup creation flag
        nullptr,                                    // user dialog procedure

        // --- Parameters specifications ---

        ParamIdBaseObject, L"base_object", TYPE_STRING, 0, IDS_BASE_OBJECT,
            p_ui, TYPE_EDITBOX, IDC_TEXTEDIT_BASE_OBJECT,
        p_end,

        ParamIdMode, L"mode", TYPE_INT, 0, IDS_MODE,
            p_ui, TYPE_INT_COMBOBOX,
                IDC_COMBO_MODE,
                2,
                IDS_MODE_FLAKES, IDS_MODE_VOXELIZATION,
            p_vals, 0, 1,
            p_default, 0,
        p_end,

        ParamIdVoxelSize, L"voxel_size", TYPE_FLOAT, 0, IDS_VOXEL_SIZE,
            p_default, 0.1f,
            p_range, 0.0f, 1.0f,
            p_ui, TYPE_SPINNER, EDITTYPE_FLOAT, IDC_EDIT_VOXEL_SIZE, IDC_SPINNER_VOXEL_SIZE, SPIN_AUTOSCALE,
        p_end,

        ParamIdFlakeSize, L"flake_size", TYPE_FLOAT, 0, IDS_FLAKE_SIZE,
            p_default, 0.05f,
            p_range, 0.0f, 1.0f,
            p_ui, TYPE_SPINNER, EDITTYPE_FLOAT, IDC_EDIT_FLAKE_SIZE, IDC_SPINNER_FLAKE_SIZE, SPIN_AUTOSCALE,
        p_end,

        ParamIdFlakeSizeJitter, L"flake_size_jitter", TYPE_FLOAT, 0, IDS_FLAKE_SIZE_JITTER,
            p_default, 0.2f,
            p_range, 0.0f, 1.0f,
            p_ui, TYPE_SPINNER, EDITTYPE_FLOAT, IDC_EDIT_FLAKE_SIZE_JITTER, IDC_SPINNER_FLAKE_SIZE_JITTER, SPIN_AUTOSCALE,
        p_end,

        ParamIdFlakeCenterJitter, L"flake_center_jitter", TYPE_FLOAT, 0, IDS_FLAKE_CENTER_JITTER,
            p_default, 0.8f,
            p_range, 0.0f, 1.0f,
            p_ui, TYPE_SPINNER, EDITTYPE_FLOAT, IDC_EDIT_FLAKE_CENTER_JITTER, IDC_SPINNER_FLAKE_CENTER_JITTER, SPIN_AUTOSCALE,
        p_end,

        ParamIdFlakesPerVoxel, L"flakes_per_voxel", TYPE_FLOAT, 0, IDS_FLAKES_PER_VOXEL,
            p_default, 1.0f,
            p_range, 0.0f, 10.0f,
            p_ui, TYPE_SPINNER, EDITTYPE_FLOAT, IDC_EDIT_FLAKES_PER_VOXEL, IDC_SPINNER_FLAKES_PER_VOXEL, SPIN_AUTOSCALE,
        p_end,

        // --- The end ---
        p_end);

    // Version of the file format.
    // Increment when making incompatible changes to the file format.
    const USHORT FileFormatVersion = 0x0001;
}

Class_ID FlakesMaxObject::get_class_id()
{
    return Class_ID(0x2172418c, 0x53b803f6);
}

FlakesMaxObject::FlakesMaxObject()
  : m_pblock(nullptr)
{
    g_flakesmaxobject_classdesc.MakeAutoParamBlocks(this);
}

void FlakesMaxObject::DeleteThis()
{
    delete this;
}

void FlakesMaxObject::GetClassName(TSTR& s)
{
    s = L"flakesMaxObject";
}

SClass_ID FlakesMaxObject::SuperClassID()
{
    return GEOMOBJECT_CLASS_ID;
}

Class_ID FlakesMaxObject::ClassID()
{
    return get_class_id();
}

void FlakesMaxObject::BeginEditParams(IObjParam* ip, ULONG flags, Animatable* prev)
{
    g_flakesmaxobject_classdesc.BeginEditParams(ip, this, flags, prev);
}

void FlakesMaxObject::EndEditParams(IObjParam* ip, ULONG flags, Animatable* next)
{
    g_flakesmaxobject_classdesc.EndEditParams(ip, this, flags, next);
}

int FlakesMaxObject::NumSubs()
{
    return NumRefs();
}

Animatable* FlakesMaxObject::SubAnim(int i)
{
    return GetReference(i);
}

TSTR FlakesMaxObject::SubAnimName(int i)
{
    return i == ParamBlockRefFlakesMaxObject ? L"Parameters" : L"";
}

int FlakesMaxObject::SubNumToRefNum(int subNum)
{
    return subNum;
}

int FlakesMaxObject::NumParamBlocks()
{
    return 1;
}

IParamBlock2* FlakesMaxObject::GetParamBlock(int i)
{
    return i == ParamBlockRefFlakesMaxObject ? m_pblock : nullptr;
}

IParamBlock2* FlakesMaxObject::GetParamBlockByID(BlockID id)
{
    return id == m_pblock->ID() ? m_pblock : nullptr;
}

int FlakesMaxObject::NumRefs()
{
    return 1;
}

RefTargetHandle FlakesMaxObject::GetReference(int i)
{
    return i == ParamBlockRefFlakesMaxObject ? m_pblock : nullptr;
}

void FlakesMaxObject::SetReference(int i, RefTargetHandle rtarg)
{
    if (i == ParamBlockRefFlakesMaxObject)
    {
        if (IParamBlock2* pblock = dynamic_cast<IParamBlock2*>(rtarg))
            m_pblock = pblock;
    }
}

RefResult FlakesMaxObject::NotifyRefChanged(
    const Interval&     changeInt,
    RefTargetHandle     hTarget,
    PartID&             partID,
    RefMessage          message,
    BOOL                propagate)
{
    return REF_SUCCEED;
}

RefTargetHandle FlakesMaxObject::Clone(RemapDir& remap)
{
    FlakesMaxObject* clone = new FlakesMaxObject();
    clone->ReplaceReference(ParamBlockRefFlakesMaxObject, remap.CloneRef(m_pblock));
    BaseClone(this, clone, remap);
    return clone;
}

BaseInterface* FlakesMaxObject::GetInterface(Interface_ID id)
{
    return
        id == IAppleseedGeometricObject::interface_id()
            ? static_cast<IAppleseedGeometricObject*>(this)
            : SimpleObject2::GetInterface(id);
}

CreateMouseCallBack* FlakesMaxObject::GetCreateMouseCallBack()
{
    return nullptr;
} 

const MCHAR* FlakesMaxObject::GetObjectName()
{
    return FlakesMaxObjectFriendlyClassName;
}

IOResult FlakesMaxObject::Save(ISave* isave)
{
    bool success = true;

    isave->BeginChunk(ChunkFileFormatVersion);
    success &= write(isave, FileFormatVersion);
    isave->EndChunk();

    isave->BeginChunk(ChunkSimpleObject2);
    success &= SimpleObject2::Save(isave) == IO_OK;
    isave->EndChunk();

    return success ? IO_OK : IO_ERROR;
}

IOResult FlakesMaxObject::Load(ILoad* iload)
{
    IOResult result = IO_OK;

    while (true)
    {
        result = iload->OpenChunk();
        if (result == IO_END)
            return IO_OK;
        if (result != IO_OK)
            break;

        switch (iload->CurChunkID())
        {
          case ChunkFileFormatVersion:
            {
                USHORT version;
                result = read<USHORT>(iload, &version);
            }
            break;

          case ChunkSimpleObject2:
            result = SimpleObject2::Load(iload);
            break;
        }

        if (result != IO_OK)
            break;

        result = iload->CloseChunk();
        if (result != IO_OK)
            break;
    }

    return result;
}

namespace
{
    static const float Vertices[] =
    {
        -0.0254550f, -0.4998350f, 0.0000000f,
        0.0254560f, -0.4998350f, 0.0000000f,
        -0.0254550f, -0.3891020f, 0.0000000f,
        0.0254560f, -0.1804750f, 0.0000000f,
        0.0254560f, -0.0440900f, 0.0000000f,
        -0.0254560f, -0.0440900f, 0.0000000f,
        -0.0254550f, -0.1804750f, 0.0000000f,
        -0.0254550f, -0.2365670f, 0.0000000f,
        0.0254560f, -0.3891020f, 0.0000000f,
        0.0254560f, -0.2365670f, 0.0000000f,
        0.1532860f, -0.4229760f, 0.0000000f,
        0.0254560f, -0.3124070f, 0.0000000f,
        -0.0254550f, -0.3124070f, 0.0000000f,
        -0.1532860f, -0.4229760f, 0.0000000f,
        -0.1153390f, -0.4668480f, 0.0000000f,
        0.1365820f, -0.2365670f, 0.0000000f,
        -0.1041970f, -0.1804750f, 0.0000000f,
        -0.1365820f, -0.2365670f, 0.0000000f,
        0.1153390f, -0.4668480f, 0.0000000f,
        0.1041970f, -0.1804750f, 0.0000000f,
        0.4201420f, -0.2719630f, 0.0000000f,
        0.4455980f, -0.2278720f, 0.0000000f,
        0.3242440f, -0.2165960f, 0.0000000f,
        0.1690240f, -0.0681920f, 0.0000000f,
        0.0509110f, 0.0000000f, 0.0000000f,
        0.1435680f, -0.1122830f, 0.0000000f,
        0.1921450f, -0.1403280f, 0.0000000f,
        0.3497000f, -0.1725060f, 0.0000000f,
        0.2176010f, -0.0962380f, 0.0000000f,
        0.4429510f, -0.0787380f, 0.0000000f,
        0.2832800f, -0.1341580f, 0.0000000f,
        0.2578240f, -0.1782480f, 0.0000000f,
        0.2896650f, -0.3442380f, 0.0000000f,
        0.3466330f, -0.3333100f, 0.0000000f,
        0.2731640f, 0.0000000f, 0.0000000f,
        0.4619710f, -0.1335370f, 0.0000000f,
        0.2083950f, 0.0000000f, 0.0000000f,
        0.4455970f, 0.2278720f, 0.0000000f,
        0.4201420f, 0.2719630f, 0.0000000f,
        0.3497000f, 0.1725060f, 0.0000000f,
        0.1435680f, 0.1122830f, 0.0000000f,
        0.0254560f, 0.0440900f, 0.0000000f,
        0.1690240f, 0.0681930f, 0.0000000f,
        0.2176010f, 0.0962380f, 0.0000000f,
        0.3242440f, 0.2165960f, 0.0000000f,
        0.1921450f, 0.1403290f, 0.0000000f,
        0.2896650f, 0.3442380f, 0.0000000f,
        0.2578240f, 0.1782490f, 0.0000000f,
        0.2832800f, 0.1341580f, 0.0000000f,
        0.4429510f, 0.0787380f, 0.0000000f,
        0.4619710f, 0.1335380f, 0.0000000f,
        0.1365820f, 0.2365670f, 0.0000000f,
        0.3466320f, 0.3333100f, 0.0000000f,
        0.1041970f, 0.1804750f, 0.0000000f,
        0.0254550f, 0.4998350f, 0.0000000f,
        -0.0254560f, 0.4998350f, 0.0000000f,
        0.0254560f, 0.3891020f, 0.0000000f,
        -0.0254560f, 0.1804750f, 0.0000000f,
        -0.0254560f, 0.0440900f, 0.0000000f,
        0.0254560f, 0.1804750f, 0.0000000f,
        0.0254550f, 0.2365670f, 0.0000000f,
        -0.0254550f, 0.3891020f, 0.0000000f,
        -0.0254550f, 0.2365670f, 0.0000000f,
        -0.1532860f, 0.4229760f, 0.0000000f,
        -0.0254550f, 0.3124070f, 0.0000000f,
        0.0254550f, 0.3124070f, 0.0000000f,
        0.1532860f, 0.4229760f, 0.0000000f,
        0.1153390f, 0.4668480f, 0.0000000f,
        -0.1365820f, 0.2365670f, 0.0000000f,
        -0.1153390f, 0.4668480f, 0.0000000f,
        -0.1041970f, 0.1804750f, 0.0000000f,
        -0.4201420f, 0.2719630f, 0.0000000f,
        -0.4455970f, 0.2278720f, 0.0000000f,
        -0.3242440f, 0.2165960f, 0.0000000f,
        -0.1690240f, 0.0681920f, 0.0000000f,
        -0.0509110f, -0.0000000f, 0.0000000f,
        -0.1435680f, 0.1122830f, 0.0000000f,
        -0.1921450f, 0.1403280f, 0.0000000f,
        -0.3497000f, 0.1725060f, 0.0000000f,
        -0.2176010f, 0.0962380f, 0.0000000f,
        -0.4429510f, 0.0787380f, 0.0000000f,
        -0.2832800f, 0.1341580f, 0.0000000f,
        -0.2578240f, 0.1782480f, 0.0000000f,
        -0.2896650f, 0.3442380f, 0.0000000f,
        -0.3466330f, 0.3333100f, 0.0000000f,
        -0.2731640f, 0.0000000f, 0.0000000f,
        -0.4619710f, 0.1335380f, 0.0000000f,
        -0.2083950f, 0.0000000f, 0.0000000f,
        -0.4455970f, -0.2278720f, 0.0000000f,
        -0.4201420f, -0.2719630f, 0.0000000f,
        -0.3497000f, -0.1725060f, 0.0000000f,
        -0.1435680f, -0.1122830f, 0.0000000f,
        -0.1690240f, -0.0681930f, 0.0000000f,
        -0.2176010f, -0.0962380f, 0.0000000f,
        -0.3242440f, -0.2165960f, 0.0000000f,
        -0.1921450f, -0.1403280f, 0.0000000f,
        -0.2896650f, -0.3442380f, 0.0000000f,
        -0.2578240f, -0.1782480f, 0.0000000f,
        -0.2832800f, -0.1341580f, 0.0000000f,
        -0.4429510f, -0.0787380f, 0.0000000f,
        -0.4619710f, -0.1335380f, 0.0000000f,
        -0.3466320f, -0.3333100f, 0.0000000f,
    };

    static const int Triangles[] =
    {
        0, 1, 2,
        3, 4, 5,
        3, 6, 7,
        2, 1, 8,
        7, 9, 3,
        8, 10, 11,
        2, 11, 12,
        11, 2, 8,
        11, 9, 7,
        11, 7, 12,
        13, 14, 2,
        3, 9, 15,
        6, 16, 17,
        2, 12, 13,
        8, 18, 10,
        19, 3, 15,
        17, 7, 6,
        6, 3, 5,
        20, 21, 22,
        23, 24, 4,
        23, 25, 26,
        22, 21, 27,
        26, 28, 23,
        27, 29, 30,
        22, 30, 31,
        30, 22, 27,
        30, 28, 26,
        30, 26, 31,
        32, 33, 22,
        23, 28, 34,
        25, 19, 15,
        22, 31, 32,
        27, 35, 29,
        36, 23, 34,
        15, 26, 25,
        25, 23, 4,
        37, 38, 39,
        40, 41, 24,
        40, 42, 43,
        39, 38, 44,
        43, 45, 40,
        44, 46, 47,
        39, 47, 48,
        47, 39, 44,
        47, 45, 43,
        47, 43, 48,
        49, 50, 39,
        40, 45, 51,
        42, 36, 34,
        39, 48, 49,
        44, 52, 46,
        53, 40, 51,
        34, 43, 42,
        42, 40, 24,
        54, 55, 56,
        57, 58, 41,
        57, 59, 60,
        56, 55, 61,
        60, 62, 57,
        61, 63, 64,
        56, 64, 65,
        64, 56, 61,
        64, 62, 60,
        64, 60, 65,
        66, 67, 56,
        57, 62, 68,
        59, 53, 51,
        56, 65, 66,
        61, 69, 63,
        70, 57, 68,
        51, 60, 59,
        59, 57, 41,
        71, 72, 73,
        74, 75, 58,
        74, 76, 77,
        73, 72, 78,
        77, 79, 74,
        78, 80, 81,
        73, 81, 82,
        81, 73, 78,
        81, 79, 77,
        81, 77, 82,
        83, 84, 73,
        74, 79, 85,
        76, 70, 68,
        73, 82, 83,
        78, 86, 80,
        87, 74, 85,
        68, 77, 76,
        76, 74, 58,
        88, 89, 90,
        91, 5, 75,
        91, 92, 93,
        90, 89, 94,
        93, 95, 91,
        94, 96, 97,
        90, 97, 98,
        97, 90, 94,
        97, 95, 93,
        97, 93, 98,
        99, 100, 90,
        91, 95, 17,
        92, 87, 85,
        90, 98, 99,
        94, 101, 96,
        16, 91, 17,
        85, 93, 92,
        92, 91, 75,
    };
}

void FlakesMaxObject::BuildMesh(TimeValue t)
{
    ivalid = FOREVER;

    const int VertexCount = countof(Vertices) / 3;
    const int TriangleCount = countof(Triangles) / 3;
    const float Size = 10.0f;

    mesh.setNumVerts(VertexCount);
    mesh.setNumFaces(TriangleCount);

    for (int i = 0; i < VertexCount; ++i)
        mesh.setVert(i, Size * Point3(Vertices[i * 3 + 0], -Vertices[i * 3 + 2], Vertices[i * 3 + 1]));

    for (int i = 0; i < TriangleCount; ++i)
        mesh.faces[i].setVerts(Triangles[i * 3 + 0], Triangles[i * 3 + 1], Triangles[i * 3 + 2]);

    mesh.InvalidateGeomCache();
}

namespace
{
    std::string make_path_relative_to_plugin(const std::string& filename)
    {
        wchar_t path[MAX_PATH];
        const DWORD path_length = sizeof(path) / sizeof(wchar_t);

        const auto result = GetModuleFileName(g_module, path, path_length);
        DbgAssert(result != 0);

        PathRemoveFileSpec(path);
        PathAppend(path, utf8_to_wide(filename).c_str());

        return wide_to_utf8(path);
    }

    bool contains_path(const asf::SearchPaths& search_paths, const bf::path path)
    {
        for (size_t i = 0, e = search_paths.get_explicit_path_count(); i < e; ++i)
        {
            if (bf::path(search_paths.get_explicit_path(i)) == path)
                return true;
        }

        return false;
    }
}

asf::auto_release_ptr<asr::Object> FlakesMaxObject::create_object(
    asr::Project&       project,
    asr::Assembly&      assembly,
    const char*         name,
    const TimeValue     time)
{
    static const char* FlakesObjectPluginPath = "flakesobject.dll";
    static const char* FlakesObjectModel = "flakes_object";

    const asr::ObjectFactoryRegistrar& object_factory_registrar = project.get_factory_registrar<asr::Object>();
    const asr::IObjectFactory* object_factory = object_factory_registrar.lookup(FlakesObjectModel);
    const std::string plugin_path = make_path_relative_to_plugin(FlakesObjectPluginPath);

    // Load the flakes plugin if it isn't available yet.
    if (object_factory == nullptr)
    {
        if (project.get_plugin_store().load_plugin(plugin_path.c_str()) == nullptr)
        {
            RENDERER_LOG_ERROR("while creating object \"%s\": failed to load plugin \"%s\".", name, plugin_path.c_str());
            return asf::auto_release_ptr<asr::Object>();
        }

        object_factory = object_factory_registrar.lookup(FlakesObjectModel);

        if (object_factory == nullptr)
        {
            RENDERER_LOG_ERROR("while creating object \"%s\": object model \"%s\" not available.", name, FlakesObjectModel);
            return asf::auto_release_ptr<asr::Object>();
        }
    }

    // Add an explicit search path to the flakes plugin to the project.
    const bf::path plugin_base_path = bf::path(plugin_path).parent_path();
    if (!contains_path(project.search_paths(), plugin_base_path))
        project.search_paths().push_back_explicit_path(plugin_base_path.string());

    asr::ParamArray params;

    const MCHAR* base_object_instance_name;
    m_pblock->GetValue(ParamIdBaseObject, time, base_object_instance_name, FOREVER);
    params.insert("base_object_instance", base_object_instance_name != nullptr ? wide_to_utf8(base_object_instance_name) : std::string());

    int mode;
    m_pblock->GetValue(ParamIdMode, time, mode, FOREVER);
    params.insert("mode", mode == 0 ? "flakes" : "voxelization");

    float voxel_size;
    m_pblock->GetValue(ParamIdMode, time, voxel_size, FOREVER);
    params.insert("voxel_size", voxel_size);

    float flake_size;
    m_pblock->GetValue(ParamIdMode, time, flake_size, FOREVER);
    params.insert("flake_size", flake_size);

    float flake_size_jitter;
    m_pblock->GetValue(ParamIdMode, time, flake_size_jitter, FOREVER);
    params.insert("flake_size_jitter", flake_size_jitter);

    float flake_center_jitter;
    m_pblock->GetValue(ParamIdMode, time, flake_center_jitter, FOREVER);
    params.insert("flake_center_jitter", flake_center_jitter);

    float flakes_per_voxel;
    m_pblock->GetValue(ParamIdMode, time, flakes_per_voxel, FOREVER);
    params.insert("flakes_per_voxel", flakes_per_voxel);

    return object_factory->create(name, params);
}


//
// FlakesMaxObjectClassDesc class implementation.
//

int FlakesMaxObjectClassDesc::IsPublic()
{
    return TRUE;
}

void* FlakesMaxObjectClassDesc::Create(BOOL loading)
{
    return new FlakesMaxObject();
}

const MCHAR* FlakesMaxObjectClassDesc::ClassName()
{
    return FlakesMaxObjectFriendlyClassName;
}

SClass_ID FlakesMaxObjectClassDesc::SuperClassID()
{
    return GEOMOBJECT_CLASS_ID;
}

Class_ID FlakesMaxObjectClassDesc::ClassID()
{
    return FlakesMaxObject::get_class_id();
}

const MCHAR* FlakesMaxObjectClassDesc::Category()
{
    return L"appleseed";
}

const MCHAR* FlakesMaxObjectClassDesc::InternalName()
{
    // Parsable name used by MAXScript.
    return L"appleseedFlakesObject";
}

HINSTANCE FlakesMaxObjectClassDesc::HInstance()
{
    return g_module;
}
