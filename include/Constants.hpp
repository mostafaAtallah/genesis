using namespace genesis;
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include "raylib.h"
#include "rlgl.h"
#include "raymath.h"
#include "box2d/box2d.h"
#include "box2d/collision.h"
#include "Tools.hpp"
#include "CompoundObject.hpp"
#include "FunctionalSystem.hpp"
#include "ResourceNode.hpp"
// #include "Constants.hpp"
namespace
{

    constexpr float kPixelsPerMeter = 60.0f;
    constexpr int kScreenWidth = 1280;
    constexpr int kScreenHeight = 720;

    Vector2 ToScreen(b2Vec2 world)
    {
        return Vector2{world.x * kPixelsPerMeter, static_cast<float>(kScreenHeight) - world.y * kPixelsPerMeter};
    }

    genesis::Vec2f ToGenesisVec2(Vector2 screen)
    {
        return genesis::Vec2f{screen.x, screen.y};
    }

    Vector3 ToRender3D(b2Vec2 world, float z = 0.0f)
    {
        return Vector3{world.x, world.y, z};
    }

    b2Vec2 ToWorld(Vector2 screen, const Camera3D &camera)
    {
        const Ray ray = GetScreenToWorldRay(screen, camera);
        const float denominator = ray.direction.z;
        if (std::abs(denominator) < 0.0001f)
        {
            return b2Vec2{ray.position.x, ray.position.y};
        }

        const float t = -ray.position.z / denominator;
        return b2Vec2{
            ray.position.x + ray.direction.x * t,
            ray.position.y + ray.direction.y * t};
    }

    float ToDegrees(b2Rot rotation)
    {
        return b2Rot_GetAngle(rotation) * RAD2DEG;
    }

    Vector2 RotateVector2(Vector2 value, float angleRadians)
    {
        const float c = std::cos(angleRadians);
        const float s = std::sin(angleRadians);
        return Vector2{
            value.x * c - value.y * s,
            value.x * s + value.y * c};
    }

    Vector3 Vector3AddV(const Vector3 &lhs, const Vector3 &rhs)
    {
        return Vector3{lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
    }

    Vector3 Vector3SubtractV(const Vector3 &lhs, const Vector3 &rhs)
    {
        return Vector3{lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
    }

    Vector3 Vector3ScaleV(const Vector3 &value, float scale)
    {
        return Vector3{value.x * scale, value.y * scale, value.z * scale};
    }

    float Vector3DotV(const Vector3 &lhs, const Vector3 &rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
    }

    Vector3 Vector3CrossV(const Vector3 &lhs, const Vector3 &rhs)
    {
        return Vector3{
            lhs.y * rhs.z - lhs.z * rhs.y,
            lhs.z * rhs.x - lhs.x * rhs.z,
            lhs.x * rhs.y - lhs.y * rhs.x};
    }

    float Vector3LengthV(const Vector3 &value)
    {
        return std::sqrt(Vector3DotV(value, value));
    }

    Vector3 Vector3NormalizeV(const Vector3 &value)
    {
        const float length = Vector3LengthV(value);
        if (length <= 0.000001f)
        {
            return Vector3{0.0f, 0.0f, 0.0f};
        }
        return Vector3ScaleV(value, 1.0f / length);
    }

    float Vector2LengthV(const Vector2 &value)
    {
        return std::sqrt(value.x * value.x + value.y * value.y);
    }

    Vector2 Vector2NormalizeV(const Vector2 &value)
    {
        const float length = Vector2LengthV(value);
        if (length <= 0.000001f)
        {
            return Vector2{0.0f, 0.0f};
        }
        return Vector2{value.x / length, value.y / length};
    }

    Vector2 Vector2Perpendicular(const Vector2 &value)
    {
        return Vector2{-value.y, value.x};
    }

    float Vector2DotV(const Vector2 &lhs, const Vector2 &rhs)
    {
        return lhs.x * rhs.x + lhs.y * rhs.y;
    }

    Vector2 Vector2AddV(const Vector2 &lhs, const Vector2 &rhs)
    {
        return Vector2{lhs.x + rhs.x, lhs.y + rhs.y};
    }

    Vector2 Vector2SubtractV(const Vector2 &lhs, const Vector2 &rhs)
    {
        return Vector2{lhs.x - rhs.x, lhs.y - rhs.y};
    }

    Vector2 Vector2ScaleV(const Vector2 &value, float scale)
    {
        return Vector2{value.x * scale, value.y * scale};
    }

    bool Vector3NearlyEqual(const Vector3 &lhs, const Vector3 &rhs, float epsilon = 0.0001f)
    {
        return std::abs(lhs.x - rhs.x) <= epsilon &&
               std::abs(lhs.y - rhs.y) <= epsilon &&
               std::abs(lhs.z - rhs.z) <= epsilon;
    }

    void AddUniquePoint(std::vector<Vector3> &points, const Vector3 &point)
    {
        for (const Vector3 &candidate : points)
        {
            if (Vector3NearlyEqual(candidate, point))
            {
                return;
            }
        }
        points.push_back(point);
    }

    struct MeshBuildData
    {
        std::vector<Vector3> vertices;
        std::vector<Vector3> normals;
    };

    void AppendTriangle(MeshBuildData &data, const Vector3 &a, const Vector3 &b, const Vector3 &c)
    {
        const Vector3 normal = Vector3NormalizeV(Vector3CrossV(Vector3SubtractV(b, a), Vector3SubtractV(c, a)));
        data.vertices.push_back(a);
        data.vertices.push_back(b);
        data.vertices.push_back(c);
        data.normals.push_back(normal);
        data.normals.push_back(normal);
        data.normals.push_back(normal);
    }

    Mesh MakeMeshFromTriangles(const MeshBuildData &data)
    {
        Mesh mesh = {0};
        mesh.vertexCount = static_cast<int>(data.vertices.size());
        mesh.triangleCount = mesh.vertexCount / 3;

        if (mesh.vertexCount <= 0)
        {
            return mesh;
        }

        mesh.vertices = static_cast<float *>(MemAlloc(static_cast<unsigned int>(sizeof(float) * data.vertices.size() * 3)));
        mesh.normals = static_cast<float *>(MemAlloc(static_cast<unsigned int>(sizeof(float) * data.normals.size() * 3)));
        mesh.texcoords = nullptr;
        mesh.texcoords2 = nullptr;
        mesh.tangents = nullptr;
        mesh.colors = nullptr;
        mesh.indices = nullptr;
        mesh.boneIndices = nullptr;
        mesh.boneWeights = nullptr;
        mesh.animVertices = nullptr;
        mesh.animNormals = nullptr;
        mesh.vaoId = 0;
        mesh.vboId = nullptr;

        for (std::size_t i = 0; i < data.vertices.size(); ++i)
        {
            mesh.vertices[i * 3 + 0] = data.vertices[i].x;
            mesh.vertices[i * 3 + 1] = data.vertices[i].y;
            mesh.vertices[i * 3 + 2] = data.vertices[i].z;
            mesh.normals[i * 3 + 0] = data.normals[i].x;
            mesh.normals[i * 3 + 1] = data.normals[i].y;
            mesh.normals[i * 3 + 2] = data.normals[i].z;
        }

        return mesh;
    }

    Vector3 IntersectSegmentPlane(const Vector3 &a, const Vector3 &b, float da, float db)
    {
        const float denominator = da - db;
        if (std::abs(denominator) <= 0.000001f)
        {
            return a;
        }

        const float t = da / denominator;
        return Vector3{
            a.x + (b.x - a.x) * t,
            a.y + (b.y - a.y) * t,
            a.z + (b.z - a.z) * t};
    }

    std::vector<Vector3> ClipPolygonAgainstPlane(
        const std::vector<Vector3> &polygon,
        const Vector3 &planePoint,
        const Vector3 &planeNormal,
        bool keepPositive,
        std::vector<Vector3> *intersections)
    {
        std::vector<Vector3> clipped;
        if (polygon.empty())
        {
            return clipped;
        }

        const auto signedDistance = [&](const Vector3 &point)
        {
            return Vector3DotV(Vector3SubtractV(point, planePoint), planeNormal);
        };
        const auto isInside = [&](float distance)
        {
            return keepPositive ? distance >= -0.0001f : distance <= 0.0001f;
        };

        Vector3 previous = polygon.back();
        float previousDistance = signedDistance(previous);
        bool previousInside = isInside(previousDistance);

        for (const Vector3 &current : polygon)
        {
            const float currentDistance = signedDistance(current);
            const bool currentInside = isInside(currentDistance);

            if (previousInside && currentInside)
            {
                clipped.push_back(current);
            }
            else if (previousInside && !currentInside)
            {
                const Vector3 intersection = IntersectSegmentPlane(previous, current, previousDistance, currentDistance);
                clipped.push_back(intersection);
                if (intersections != nullptr)
                {
                    intersections->push_back(intersection);
                }
            }
            else if (!previousInside && currentInside)
            {
                const Vector3 intersection = IntersectSegmentPlane(previous, current, previousDistance, currentDistance);
                clipped.push_back(intersection);
                clipped.push_back(current);
                if (intersections != nullptr)
                {
                    intersections->push_back(intersection);
                }
            }

            previous = current;
            previousDistance = currentDistance;
            previousInside = currentInside;
        }

        return clipped;
    }

    void TriangulatePolygonFan(const std::vector<Vector3> &polygon, MeshBuildData &data)
    {
        if (polygon.size() < 3)
        {
            return;
        }

        for (std::size_t i = 1; i + 1 < polygon.size(); ++i)
        {
            AppendTriangle(data, polygon[0], polygon[i], polygon[i + 1]);
        }
    }

    void AddCapTriangles(const std::vector<Vector3> &points, const Vector3 &planeNormal, MeshBuildData &data, bool flipWinding)
    {
        if (points.size() < 3)
        {
            return;
        }

        Vector3 centroid{0.0f, 0.0f, 0.0f};
        for (const Vector3 &point : points)
        {
            centroid = Vector3AddV(centroid, point);
        }
        centroid = Vector3ScaleV(centroid, 1.0f / static_cast<float>(points.size()));

        Vector3 reference = std::abs(planeNormal.z) < 0.99f ? Vector3{0.0f, 0.0f, 1.0f} : Vector3{0.0f, 1.0f, 0.0f};
        Vector3 basisU = Vector3NormalizeV(Vector3CrossV(reference, planeNormal));
        if (Vector3LengthV(basisU) <= 0.000001f)
        {
            reference = Vector3{1.0f, 0.0f, 0.0f};
            basisU = Vector3NormalizeV(Vector3CrossV(reference, planeNormal));
        }
        const Vector3 basisV = Vector3CrossV(planeNormal, basisU);

        std::vector<Vector3> ordered = points;
        std::sort(ordered.begin(), ordered.end(), [&](const Vector3 &lhs, const Vector3 &rhs)
                  {
        const Vector3 lhsOffset = Vector3SubtractV(lhs, centroid);
        const Vector3 rhsOffset = Vector3SubtractV(rhs, centroid);
        const float lhsAngle = std::atan2(Vector3DotV(lhsOffset, basisV), Vector3DotV(lhsOffset, basisU));
        const float rhsAngle = std::atan2(Vector3DotV(rhsOffset, basisV), Vector3DotV(rhsOffset, basisU));
        return lhsAngle < rhsAngle; });

        for (std::size_t i = 0; i < ordered.size(); ++i)
        {
            AddUniquePoint(ordered, ordered[i]);
        }

        for (std::size_t i = 0; i < ordered.size(); ++i)
        {
            const std::size_t next = (i + 1) % ordered.size();
            if (!flipWinding)
            {
                AppendTriangle(data, centroid, ordered[i], ordered[next]);
            }
            else
            {
                AppendTriangle(data, centroid, ordered[next], ordered[i]);
            }
        }
    }

    struct SliceSideData
    {
        MeshBuildData geometry;
        std::vector<Vector3> capPoints;
    };

    Mesh BuildMeshFromSideData(const SliceSideData &data)
    {
        return MakeMeshFromTriangles(data.geometry);
    }

    Vector3 ComputeAveragePoint(const std::vector<Vector3> &points)
    {
        Vector3 result{0.0f, 0.0f, 0.0f};
        if (points.empty())
        {
            return result;
        }

        for (const Vector3 &point : points)
        {
            result = Vector3AddV(result, point);
        }

        return Vector3ScaleV(result, 1.0f / static_cast<float>(points.size()));
    }

    std::vector<Vector3> ExtractUniqueVertices(const Mesh &mesh)
    {
        std::vector<Vector3> points;
        for (int i = 0; i < mesh.vertexCount; ++i)
        {
            const Vector3 point{
                mesh.vertices[i * 3 + 0],
                mesh.vertices[i * 3 + 1],
                mesh.vertices[i * 3 + 2]};
            AddUniquePoint(points, point);
        }
        return points;
    }

    Mesh CloneMesh(const Mesh &source)
    {
        Mesh clone = source;
        clone.vboId = nullptr;
        clone.vaoId = 0;

        if (source.vertexCount > 0 && source.vertices != nullptr)
        {
            clone.vertices = static_cast<float *>(MemAlloc(sizeof(float) * static_cast<std::size_t>(source.vertexCount) * 3));
            std::copy(source.vertices, source.vertices + static_cast<std::size_t>(source.vertexCount) * 3, clone.vertices);
        }
        if (source.normals != nullptr)
        {
            clone.normals = static_cast<float *>(MemAlloc(sizeof(float) * static_cast<std::size_t>(source.vertexCount) * 3));
            std::copy(source.normals, source.normals + static_cast<std::size_t>(source.vertexCount) * 3, clone.normals);
        }
        if (source.texcoords != nullptr)
        {
            clone.texcoords = static_cast<float *>(MemAlloc(sizeof(float) * static_cast<std::size_t>(source.vertexCount) * 2));
            std::copy(source.texcoords, source.texcoords + static_cast<std::size_t>(source.vertexCount) * 2, clone.texcoords);
        }
        if (source.indices != nullptr && source.triangleCount > 0)
        {
            clone.indices = static_cast<unsigned short *>(MemAlloc(sizeof(unsigned short) * static_cast<std::size_t>(source.triangleCount) * 3));
            std::copy(source.indices, source.indices + static_cast<std::size_t>(source.triangleCount) * 3, clone.indices);
        }

        return clone;
    }

    // tools were here
    Material &DefaultDrawMaterial()
    {
        static Material material = LoadMaterialDefault();
        return material;
    }

    bool IsBrownish(const genesis::Component &component)
    {
        return component.material.vector[0] > component.material.vector[1] + 0.04f &&
               component.material.vector[0] > component.material.vector[2] + 0.04f;
    }

    bool IsHandleLike(const genesis::Component &component, const Vector3 &sizeMeters)
    {
        const float longest = std::max(sizeMeters.x, sizeMeters.y);
        const float shortest = std::max(std::min(sizeMeters.x, sizeMeters.y), 0.001f);
        return (longest / shortest) >= 2.2f && (IsBrownish(component) || component.name == "grip" || component.name == "log");
    }

    bool PointInPolygon2D(const std::vector<Vector2> &polygon, const Vector2 &point)
    {
        bool inside = false;
        if (polygon.size() < 3)
        {
            return false;
        }

        for (std::size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
        {
            const Vector2 &a = polygon[i];
            const Vector2 &b = polygon[j];
            const bool intersects = ((a.y > point.y) != (b.y > point.y)) &&
                                    (point.x < (b.x - a.x) * (point.y - a.y) / ((b.y - a.y) + 0.000001f) + a.x);
            if (intersects)
            {
                inside = !inside;
            }
        }

        return inside;
    }

    std::vector<Vector2> BuildRibbonPolygon(const std::vector<Vector2> &stroke, float halfWidth)
    {
        std::vector<Vector2> polygon;
        if (stroke.size() < 2)
        {
            return polygon;
        }

        std::vector<Vector2> leftSide;
        std::vector<Vector2> rightSide;
        leftSide.reserve(stroke.size());
        rightSide.reserve(stroke.size());

        for (std::size_t i = 0; i < stroke.size(); ++i)
        {
            const Vector2 previous = i == 0 ? stroke[i] : stroke[i - 1];
            const Vector2 next = i + 1 < stroke.size() ? stroke[i + 1] : stroke[i];
            Vector2 tangent = Vector2NormalizeV(Vector2SubtractV(next, previous));
            if (Vector2LengthV(tangent) < 0.000001f)
            {
                tangent = Vector2{1.0f, 0.0f};
            }
            const Vector2 normal = Vector2NormalizeV(Vector2Perpendicular(tangent));
            leftSide.push_back(Vector2AddV(stroke[i], Vector2ScaleV(normal, halfWidth)));
            rightSide.push_back(Vector2SubtractV(stroke[i], Vector2ScaleV(normal, halfWidth)));
        }

        polygon.insert(polygon.end(), leftSide.begin(), leftSide.end());
        for (auto it = rightSide.rbegin(); it != rightSide.rend(); ++it)
        {
            polygon.push_back(*it);
        }

        return polygon;
    }

    Mesh BuildPrismMeshFromPolygon(const std::vector<Vector2> &polygon, float depth)
    {
        MeshBuildData data;
        if (polygon.size() < 3)
        {
            return MakeMeshFromTriangles(data);
        }

        const float zTop = depth * 0.5f;
        const float zBottom = -depth * 0.5f;
        Vector2 centroid2D{0.0f, 0.0f};
        for (const Vector2 &point : polygon)
        {
            centroid2D = Vector2AddV(centroid2D, point);
        }
        centroid2D = Vector2ScaleV(centroid2D, 1.0f / static_cast<float>(polygon.size()));

        const Vector3 topCenter{centroid2D.x, centroid2D.y, zTop};
        const Vector3 bottomCenter{centroid2D.x, centroid2D.y, zBottom};
        std::vector<Vector3> topPoints;
        std::vector<Vector3> bottomPoints;
        topPoints.reserve(polygon.size());
        bottomPoints.reserve(polygon.size());

        for (const Vector2 &point : polygon)
        {
            topPoints.push_back(Vector3{point.x, point.y, zTop});
            bottomPoints.push_back(Vector3{point.x, point.y, zBottom});
        }

        TriangulatePolygonFan(topPoints, data);
        for (std::size_t i = 1; i + 1 < bottomPoints.size(); ++i)
        {
            AppendTriangle(data, bottomCenter, bottomPoints[i + 1], bottomPoints[i]);
        }

        for (std::size_t i = 0; i < polygon.size(); ++i)
        {
            const std::size_t next = (i + 1) % polygon.size();
            const Vector3 aTop = topPoints[i];
            const Vector3 bTop = topPoints[next];
            const Vector3 aBottom = bottomPoints[i];
            const Vector3 bBottom = bottomPoints[next];
            AppendTriangle(data, aTop, bTop, bBottom);
            AppendTriangle(data, aTop, bBottom, aBottom);
        }

        return MakeMeshFromTriangles(data);
    }

    Mesh BuildComponentMesh(const genesis::Component &component)
    {
        const float depth = component.dimensions[2] > 0.0001f ? component.dimensions[2] : 0.55f;
        return GenMeshCube(component.size.x / kPixelsPerMeter, component.size.y / kPixelsPerMeter, depth);
    }

    void SliceComponentMesh(Mesh &mesh, Vector3 planePoint, Vector3 planeNormal, Mesh &outMeshA, Mesh &outMeshB)
    {
        outMeshA = Mesh{0};
        outMeshB = Mesh{0};

        if (mesh.vertexCount < 3 || mesh.vertices == nullptr)
        {
            return;
        }

        const Vector3 planeUnitNormal = Vector3NormalizeV(planeNormal);
        if (Vector3LengthV(planeUnitNormal) <= 0.000001f)
        {
            return;
        }

        SliceSideData sideA;
        SliceSideData sideB;
        std::vector<Vector3> intersectionPoints;

        auto readVertex = [&](int vertexIndex)
        {
            return Vector3{
                mesh.vertices[vertexIndex * 3 + 0],
                mesh.vertices[vertexIndex * 3 + 1],
                mesh.vertices[vertexIndex * 3 + 2]};
        };

        const bool indexed = mesh.indices != nullptr;
        const int triangleCount = mesh.triangleCount;
        for (int triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex)
        {
            std::array<Vector3, 3> triangle{};
            if (indexed)
            {
                triangle[0] = readVertex(mesh.indices[triangleIndex * 3 + 0]);
                triangle[1] = readVertex(mesh.indices[triangleIndex * 3 + 1]);
                triangle[2] = readVertex(mesh.indices[triangleIndex * 3 + 2]);
            }
            else
            {
                triangle[0] = readVertex(triangleIndex * 3 + 0);
                triangle[1] = readVertex(triangleIndex * 3 + 1);
                triangle[2] = readVertex(triangleIndex * 3 + 2);
            }

            std::vector<Vector3> triVertices = {triangle[0], triangle[1], triangle[2]};
            std::vector<Vector3> clippedPositive = ClipPolygonAgainstPlane(triVertices, planePoint, planeUnitNormal, true, &intersectionPoints);
            std::vector<Vector3> clippedNegative = ClipPolygonAgainstPlane(triVertices, planePoint, planeUnitNormal, false, &intersectionPoints);

            TriangulatePolygonFan(clippedPositive, sideA.geometry);
            TriangulatePolygonFan(clippedNegative, sideB.geometry);
        }

        std::vector<Vector3> capPoints;
        for (const Vector3 &point : intersectionPoints)
        {
            AddUniquePoint(capPoints, point);
        }

        if (capPoints.size() >= 3)
        {
            AddCapTriangles(capPoints, planeUnitNormal, sideA.geometry, false);
            AddCapTriangles(capPoints, planeUnitNormal, sideB.geometry, true);
        }

        outMeshA = MakeMeshFromTriangles(sideA.geometry);
        outMeshB = MakeMeshFromTriangles(sideB.geometry);
    }

    Color MaterialColor(const genesis::Component &component)
    {
        const auto clamp_channel = [](float value) -> unsigned char
        {
            const float normalized = std::clamp(value, 0.0f, 1.0f);
            return static_cast<unsigned char>(normalized * 180.0f + 40.0f);
        };

        return Color{
            clamp_channel(component.material.vector[0]),
            clamp_channel(component.material.vector[1]),
            clamp_channel(component.material.vector[2]),
            255};
    }

    void BeginBodyTransform(const Vector3 &position, float angleDegrees)
    {
        rlPushMatrix();
        rlTranslatef(position.x, position.y, position.z);
        rlRotatef(angleDegrees, 0.0f, 0.0f, 1.0f);
    }

    void EndBodyTransform()
    {
        rlPopMatrix();
    }

    inline void ReleaseMeshData(Mesh &mesh)
    {
        if (mesh.vertices != nullptr || mesh.normals != nullptr || mesh.indices != nullptr || mesh.texcoords != nullptr)
        {
            UnloadMesh(mesh);
        }
        mesh = Mesh{0};
    }

    inline void ReleaseToolPartMesh(genesis::ToolPartRuntime &part) {
        if (part.ownsMesh)
        {
            ReleaseMeshData(part.mesh);
            part.ownsMesh = false;
        }
    }

    struct SimulationState{
        b2WorldId worldId = b2_nullWorldId;
        b2BodyId floorId = b2_nullBodyId;
        b2BodyId leftWallId = b2_nullBodyId;
        b2BodyId rightWallId = b2_nullBodyId;
        b2BodyId cursorBodyId = b2_nullBodyId;
        b2JointId dragJointId = b2_nullJointId;
        BodyTag cursorTag{BodyTag::Kind::Cursor, 0};
        std::vector<ToolPartRuntime> parts;
        std::vector<ToolLinkRuntime> joints;
        std::vector<ResourceRuntime> resources;
        genesis::CompoundObject toolModel{"active_tool"};
        genesis::FunctionalSystem functionalSystem;
        bool dragging = false;
        int grabbedPart = -1;
        bool workbenchActive = false;
        bool swinging = false;
        float lastImpactForce = 0.0f;
        float lastJointStress = 0.0f;
        float simulationTime = 0.0f;
        int workbenchSelection = -1;
        bool workbenchDragging = false;
        b2Vec2 workbenchDragOffset{0.0f, 0.0f};
        std::string activeResourceName;
        float activeResourceHardness = 0.0f;
        float activeResourceDurability = 0.0f;
        std::string lastDroppedItem;
    };

    b2BodyId CreateBoxBody(b2WorldId worldId, b2BodyType type, b2Vec2 position, b2Vec2 halfExtents, bool contactEvents, bool hitEvents)
    {
        b2BodyDef bodyDef = b2DefaultBodyDef();
        bodyDef.type = type;
        bodyDef.position = position;
        b2BodyId bodyId = b2CreateBody(worldId, &bodyDef);

        b2ShapeDef shapeDef = b2DefaultShapeDef();
        shapeDef.enableContactEvents = contactEvents;
        shapeDef.enableHitEvents = hitEvents;

        b2Polygon box = b2MakeBox(halfExtents.x, halfExtents.y);
        b2CreatePolygonShape(bodyId, &shapeDef, &box);
        return bodyId;
    }

    void SyncComponentVisual(genesis::Component &component, b2BodyId bodyId)
    {
        const b2Vec2 position = b2Body_GetPosition(bodyId);
        component.position = ToGenesisVec2(ToScreen(position));
    }

    void DestroyDragJoint(SimulationState &sim)
    {
        if (b2Joint_IsValid(sim.dragJointId))
        {
            b2DestroyJoint(sim.dragJointId, true);
        }

        sim.dragJointId = b2_nullJointId;
        sim.cursorBodyId = b2_nullBodyId;
        sim.dragging = false;
        sim.grabbedPart = -1;
    }

    void SetBodyWorkbenchState(b2BodyId bodyId, bool frozen)
    {
        if (!b2Body_IsValid(bodyId))
        {
            return;
        }

        b2Body_SetType(bodyId, frozen ? b2_kinematicBody : b2_dynamicBody);
        b2Body_SetGravityScale(bodyId, frozen ? 0.0f : 1.0f);
        b2Body_SetLinearVelocity(bodyId, b2Vec2{0.0f, 0.0f});
        b2Body_SetAngularVelocity(bodyId, 0.0f);
        b2Body_SetAwake(bodyId, true);
    }

    void ClearWorkbenchSelection(SimulationState &sim)
    {
        if (sim.workbenchSelection >= 0 && sim.workbenchSelection < static_cast<int>(sim.parts.size()))
        {
            SetBodyWorkbenchState(sim.parts[static_cast<std::size_t>(sim.workbenchSelection)].bodyId, false);
        }

        sim.workbenchSelection = -1;
        sim.workbenchDragging = false;
        sim.workbenchDragOffset = b2Vec2{0.0f, 0.0f};
    }

    void SelectWorkbenchPart(SimulationState &sim, int index)
    {
        if (index < 0 || index >= static_cast<int>(sim.parts.size()))
        {
            ClearWorkbenchSelection(sim);
            return;
        }

        if (sim.workbenchSelection != index)
        {
            ClearWorkbenchSelection(sim);
            sim.workbenchSelection = index;
            SetBodyWorkbenchState(sim.parts[static_cast<std::size_t>(index)].bodyId, true);
        }
    }

    int FindWorkbenchPartAtMouse(const SimulationState &sim, const b2Vec2 &mouseWorld)
    {
        for (int i = static_cast<int>(sim.parts.size()) - 1; i >= 0; --i)
        {
            if (b2Shape_TestPoint(sim.parts[static_cast<std::size_t>(i)].shapeId, mouseWorld))
            {
                return i;
            }
        }

        return -1;
    }

    void MoveSelectedWorkbenchPart(SimulationState &sim, const b2Vec2 &mouseWorld, bool mouseDown, float dt)
    {
        if (sim.workbenchSelection < 0 || sim.workbenchSelection >= static_cast<int>(sim.parts.size()))
        {
            return;
        }

        auto &part = sim.parts[static_cast<std::size_t>(sim.workbenchSelection)];
        if (!b2Body_IsValid(part.bodyId))
        {
            return;
        }

        if (mouseDown)
        {
            if (!sim.workbenchDragging)
            {
                sim.workbenchDragging = true;
                sim.workbenchDragOffset = b2Body_GetPosition(part.bodyId) - mouseWorld;
            }

            const b2Vec2 targetPosition = mouseWorld + sim.workbenchDragOffset;
            b2Transform target = {targetPosition, b2Body_GetRotation(part.bodyId)};
            b2Body_SetTargetTransform(part.bodyId, target, dt, true);
            b2Body_SetLinearVelocity(part.bodyId, b2Vec2{0.0f, 0.0f});
            b2Body_SetAngularVelocity(part.bodyId, 0.0f);
        }
        else
        {
            sim.workbenchDragging = false;
        }
    }

    void RebindToolPartUserData(SimulationState &sim)
    {
        for (std::size_t i = 0; i < sim.parts.size(); ++i)
        {
            sim.parts[i].tag = {BodyTag::Kind::ToolPart, i};
            if (b2Body_IsValid(sim.parts[i].bodyId))
            {
                b2Body_SetUserData(sim.parts[i].bodyId, &sim.parts[i].tag);
            }
            if (b2Shape_IsValid(sim.parts[i].shapeId))
            {
                b2Shape_SetUserData(sim.parts[i].shapeId, &sim.parts[i].tag);
            }
        }
    }

    void RebuildToolModelFromParts(SimulationState &sim)
    {
        sim.toolModel = genesis::CompoundObject{"active_tool"};
        for (const auto &part : sim.parts)
        {
            sim.toolModel.AddComponent(part.component);
        }
    }

    struct CutBodyBuildResult{
        ToolPartRuntime part;
        bool valid = false;
    };

    CutBodyBuildResult BuildCutPartFromMesh(
        const ToolPartRuntime &sourcePart,
        b2WorldId worldId,
        const Mesh &mesh,
        b2Vec2 bodyWorldPosition,
        float bodyAngleRadians,
        const std::string &suffix)
    {
        CutBodyBuildResult result;
        if (mesh.vertexCount < 3)
        {
            return result;
        }

        auto component = std::make_shared<genesis::Component>(*sourcePart.component);
        const BoundingBox bounds = GetMeshBoundingBox(mesh);
        const Vector3 sizeMeters{
            std::max(bounds.max.x - bounds.min.x, 0.05f),
            std::max(bounds.max.y - bounds.min.y, 0.05f),
            std::max(bounds.max.z - bounds.min.z, 0.05f)};

        component->size = genesis::Vec2f{
            sizeMeters.x * kPixelsPerMeter,
            sizeMeters.y * kPixelsPerMeter};
        component->dimensions = {sizeMeters.x, sizeMeters.y, sizeMeters.z};
        if (IsHandleLike(*component, sizeMeters))
        {
            component->name = "Handle Shaft";
        }
        else
        {
            component->name = sourcePart.component->name + suffix;
        }

        const std::vector<Vector3> uniqueVertices = ExtractUniqueVertices(mesh);
        const Vector3 localCenter = ComputeAveragePoint(uniqueVertices);
        std::vector<b2Vec2> hullPoints;
        hullPoints.reserve(uniqueVertices.size());
        for (const Vector3 &vertex : uniqueVertices)
        {
            hullPoints.push_back(b2Vec2{vertex.x - localCenter.x, vertex.y - localCenter.y});
        }

        ToolPartRuntime part;
        part.component = component;
        part.displayDepth = sourcePart.displayDepth;
        part.tag = {BodyTag::Kind::ToolPart, 0};

        b2BodyDef bodyDef = b2DefaultBodyDef();
        bodyDef.type = b2_dynamicBody;
        bodyDef.position = bodyWorldPosition;
        bodyDef.rotation = b2MakeRot(bodyAngleRadians);
        bodyDef.linearDamping = 0.08f;
        bodyDef.angularDamping = 1.0f;
        part.bodyId = b2CreateBody(worldId, &bodyDef);

        b2ShapeDef shapeDef = b2DefaultShapeDef();
        shapeDef.density = component->material.density;
        shapeDef.material.friction = 0.55f;
        shapeDef.material.restitution = 0.08f;
        shapeDef.enableContactEvents = true;
        shapeDef.enableHitEvents = true;

        b2Polygon polygon = b2MakeBox(std::max(sizeMeters.x * 0.5f, 0.05f), std::max(sizeMeters.y * 0.5f, 0.05f));
        if (hullPoints.size() >= 3)
        {
            b2Hull hull = b2ComputeHull(hullPoints.data(), static_cast<int>(hullPoints.size()));
            if (b2ValidateHull(&hull))
            {
                polygon = b2MakePolygon(&hull, 0.01f);
            }
        }

        part.shapeId = b2CreatePolygonShape(part.bodyId, &shapeDef, &polygon);
        b2Body_SetUserData(part.bodyId, &part.tag);
        b2Shape_SetUserData(part.shapeId, &part.tag);
        part.mesh = CloneMesh(mesh);
        UploadMesh(&part.mesh, false);
        part.ownsMesh = true;
        result.part = part;
        result.valid = b2Body_IsValid(part.bodyId) && b2Shape_IsValid(part.shapeId);

        return result;
    }

    bool CutSelectedWorkbenchPart(SimulationState &sim, const Vector2 &mouseWorld2D)
    {
        if (!sim.workbenchActive || sim.workbenchSelection < 0 || sim.workbenchSelection >= static_cast<int>(sim.parts.size()))
        {
            return false;
        }

        const std::size_t selectedIndex = static_cast<std::size_t>(sim.workbenchSelection);
        auto &selectedPart = sim.parts[selectedIndex];
        if (!b2Body_IsValid(selectedPart.bodyId))
        {
            return false;
        }

        const b2Vec2 bodyPosition = b2Body_GetPosition(selectedPart.bodyId);
        const float bodyAngle = b2Rot_GetAngle(b2Body_GetRotation(selectedPart.bodyId));
        Vector2 sliceDirection = Vector2{mouseWorld2D.x - bodyPosition.x, mouseWorld2D.y - bodyPosition.y};
        if (Vector2LengthV(sliceDirection) < 0.001f)
        {
            sliceDirection = Vector2{1.0f, 0.0f};
        }

        Vector2 planeNormalWorld2D = Vector2NormalizeV(sliceDirection);
        Vector2 planePointWorld2D = mouseWorld2D;
        const Vector2 planeOffset = RotateVector2(Vector2{planePointWorld2D.x - bodyPosition.x, planePointWorld2D.y - bodyPosition.y}, -bodyAngle);
        const Vector2 normalOffset = RotateVector2(planeNormalWorld2D, -bodyAngle);
        const Vector3 localPlanePoint{planeOffset.x, planeOffset.y, 0.0f};
        const Vector3 localPlaneNormal{normalOffset.x, normalOffset.y, 0.0f};

        Mesh sourceMesh = BuildComponentMesh(*selectedPart.component);
        Mesh meshA = {0};
        Mesh meshB = {0};
        SliceComponentMesh(sourceMesh, localPlanePoint, localPlaneNormal, meshA, meshB);
        UnloadMesh(sourceMesh);

        if (meshA.vertexCount < 3 || meshB.vertexCount < 3)
        {
            if (meshA.vertexCount > 0)
            {
                UnloadMesh(meshA);
            }
            if (meshB.vertexCount > 0)
            {
                UnloadMesh(meshB);
            }
            sim.lastDroppedItem = "Cut failed";
            return false;
        }

        const std::vector<Vector3> uniqueA = ExtractUniqueVertices(meshA);
        const std::vector<Vector3> uniqueB = ExtractUniqueVertices(meshB);
        const Vector3 centerA = ComputeAveragePoint(uniqueA);
        const Vector3 centerB = ComputeAveragePoint(uniqueB);
        const Vector2 worldCenterA2D = RotateVector2(Vector2{centerA.x, centerA.y}, bodyAngle);
        const Vector2 worldCenterB2D = RotateVector2(Vector2{centerB.x, centerB.y}, bodyAngle);

        CutBodyBuildResult pieceA = BuildCutPartFromMesh(
            selectedPart,
            sim.worldId,
            meshA,
            b2Vec2{bodyPosition.x + worldCenterA2D.x, bodyPosition.y + worldCenterA2D.y},
            bodyAngle,
            " A");

        CutBodyBuildResult pieceB = BuildCutPartFromMesh(
            selectedPart,
            sim.worldId,
            meshB,
            b2Vec2{bodyPosition.x + worldCenterB2D.x, bodyPosition.y + worldCenterB2D.y},
            bodyAngle,
            " B");

        UnloadMesh(meshA);
        UnloadMesh(meshB);

        if (!pieceA.valid || !pieceB.valid)
        {
            if (pieceA.valid && b2Body_IsValid(pieceA.part.bodyId))
            {
                b2DestroyBody(pieceA.part.bodyId);
            }
            if (pieceB.valid && b2Body_IsValid(pieceB.part.bodyId))
            {
                b2DestroyBody(pieceB.part.bodyId);
            }
            sim.lastDroppedItem = "Cut failed";
            return false;
        }

        DestroyDragJoint(sim);
        ClearWorkbenchSelection(sim);
        if (b2Body_IsValid(selectedPart.bodyId))
        {
            b2DestroyBody(selectedPart.bodyId);
        }

        sim.parts.erase(sim.parts.begin() + static_cast<std::ptrdiff_t>(selectedIndex));
        sim.parts.insert(sim.parts.begin() + static_cast<std::ptrdiff_t>(selectedIndex), pieceB.part);
        sim.parts.insert(sim.parts.begin() + static_cast<std::ptrdiff_t>(selectedIndex), pieceA.part);

        RebindToolPartUserData(sim);
        RebuildToolModelFromParts(sim);
        SelectWorkbenchPart(sim, static_cast<int>(selectedIndex));

        sim.lastDroppedItem = TextFormat("Cut: %s", sim.parts[selectedIndex].component->name.c_str());
        return true;
    }

    void ApplyImpactToTool(SimulationState &sim, std::size_t struckPartIndex, float normalImpulse)
    {
        sim.lastImpactForce = normalImpulse;

        if (struckPartIndex >= sim.parts.size())
        {
            return;
        }

        float propagatedImpulse = normalImpulse;
        for (int jointIndex = static_cast<int>(struckPartIndex) - 1; jointIndex >= 0; --jointIndex)
        {
            auto &joint = sim.joints[static_cast<std::size_t>(jointIndex)];
            if (joint.broken || !b2Joint_IsValid(joint.jointId))
            {
                continue;
            }

            const float jointStress = propagatedImpulse * (joint.interlocking ? 0.25f : 1.3f);
            sim.lastJointStress = jointStress;

            if (jointStress > joint.maxShearForce)
            {
                b2DestroyJoint(joint.jointId, true);
                joint.broken = true;
            }

            propagatedImpulse *= 0.75f;
        }

        sim.toolModel.ApplyStrain(normalImpulse * 0.0015f);
    }

    void DrawToolPart(const ToolPartRuntime &part, bool selected, const Camera3D &camera)
    {
        const genesis::Component &component = *part.component;
        const Color fill = selected ? Color{246, 204, 88, 255} : MaterialColor(component);
        const Vector3 position3d = ToRender3D(b2Body_GetPosition(part.bodyId), part.displayDepth);
        const Vector3 size3d{
            component.size.x / kPixelsPerMeter,
            component.size.y / kPixelsPerMeter,
            component.dimensions[2] > 0.0001f ? component.dimensions[2] : 0.55f};
        const float angle = ToDegrees(b2Body_GetRotation(part.bodyId));

        const Vector3 shadowPos = Vector3{position3d.x + 0.14f, 0.03f, position3d.z - 0.38f};
        DrawCubeV(shadowPos, Vector3{size3d.x * 1.02f, size3d.y * 0.95f, 0.02f}, Color{0, 0, 0, 48});

        BeginBodyTransform(position3d, angle);
        const Material &material = DefaultDrawMaterial();
        if (part.mesh.vertexCount > 0)
        {
            DrawMesh(part.mesh, material, MatrixIdentity());
        }
        else
        {
            DrawCubeV(Vector3{0.0f, 0.0f, 0.0f}, size3d, fill);
        }
        DrawCubeWiresV(Vector3{0.0f, 0.0f, 0.0f}, size3d, selected ? Color{255, 240, 180, 255} : Color{220, 230, 240, 255});
        EndBodyTransform();

        const Vector2 labelPos = GetWorldToScreen(Vector3{position3d.x, position3d.y + 0.45f, position3d.z}, camera);
        DrawText(component.name.c_str(), static_cast<int>(labelPos.x - 18.0f), static_cast<int>(labelPos.y), 18, RAYWHITE);
    }

    void DrawResourceNode(const ResourceRuntime &resource, const Camera3D &camera)
    {
        if (resource.node.IsHarvested() || !b2Body_IsValid(resource.bodyId))
        {
            return;
        }

        const Vector3 position3d = ToRender3D(b2Body_GetPosition(resource.bodyId), resource.displayDepth);
        const genesis::Vec2f &size = resource.node.Size();
        const Vector3 size3d{
            size.x,
            size.y,
            0.7f};
        const float angle = ToDegrees(b2Body_GetRotation(resource.bodyId));

        const Vector3 shadowPos = Vector3{position3d.x + 0.18f, 0.03f, position3d.z - 0.46f};
        DrawCubeV(shadowPos, Vector3{size3d.x * 1.02f, size3d.y * 0.98f, 0.02f}, Color{0, 0, 0, 56});

        BeginBodyTransform(position3d, angle);
        DrawCubeV(Vector3{0.0f, 0.0f, 0.0f}, size3d, resource.node.DisplayColor());
        DrawCubeWiresV(Vector3{0.0f, 0.0f, 0.0f}, size3d, Color{25, 28, 34, 255});
        EndBodyTransform();

        const Vector2 labelPos = GetWorldToScreen(Vector3{position3d.x, position3d.y + size.y * 0.55f, position3d.z}, camera);
        DrawText(resource.node.Name().c_str(), static_cast<int>(labelPos.x - 26.0f), static_cast<int>(labelPos.y), 18, Color{25, 28, 34, 255});
    }

    void DestroySimulation(SimulationState &sim)
    {
        using namespace genesis;

        DestroyDragJoint(sim);

        if (b2World_IsValid(sim.worldId))
        {
            b2DestroyWorld(sim.worldId);
            sim.worldId = b2_nullWorldId;
        }

        for (auto &part : sim.parts)
        {
            ReleaseToolPartMesh(part);
        }
        sim.floorId = b2_nullBodyId;
        sim.leftWallId = b2_nullBodyId;
        sim.rightWallId = b2_nullBodyId;
        sim.parts.clear();
        sim.joints.clear();
        sim.resources.clear();
        sim.toolModel = CompoundObject{"active_tool"};
        sim.functionalSystem = FunctionalSystem{};
        sim.workbenchActive = false;
        sim.workbenchSelection = -1;
        sim.workbenchDragging = false;
        sim.swinging = false;
        sim.lastImpactForce = 0.0f;
        sim.lastJointStress = 0.0f;
        sim.simulationTime = 0.0f;
        sim.activeResourceName.clear();
        sim.activeResourceHardness = 0.0f;
        sim.activeResourceDurability = 0.0f;
        sim.lastDroppedItem.clear();
    }

    void BuildSimulation(SimulationState &sim)
    {
        using namespace genesis;

        sim.toolModel = CompoundObject{"active_tool"};
        sim.functionalSystem = FunctionalSystem{};

        b2WorldDef worldDef = b2DefaultWorldDef();
        worldDef.gravity = b2Vec2{0.0f, -18.0f};
        worldDef.hitEventThreshold = 1.0f;
        sim.worldId = b2CreateWorld(&worldDef);

        auto grip = std::make_shared<Component>();
        grip->name = "grip";
        grip->size = {150.0f, 70.0f};
        grip->material.vector = {0.58f, 0.48f, 0.40f};
        grip->material.density = 1.2f;
        grip->material.hardness = 0.36f;
        grip->material.elasticity = 0.28f;
        grip->topology = ComponentTopology::PiercedHollow;
        grip->dimensions = {grip->size.x / kPixelsPerMeter, grip->size.y / kPixelsPerMeter, 0.55f};

        auto shaft = std::make_shared<Component>();
        shaft->name = "shaft";
        shaft->size = {100.0f, 46.0f};
        shaft->material.vector = {0.75f, 0.68f, 0.52f};
        shaft->material.density = 1.8f;
        shaft->material.hardness = 0.62f;
        shaft->material.elasticity = 0.22f;
        shaft->topology = ComponentTopology::PrimitiveSolid;
        shaft->dimensions = {shaft->size.x / kPixelsPerMeter, shaft->size.y / kPixelsPerMeter, 0.55f};

        auto head = std::make_shared<Component>();
        head->name = "head";
        head->size = {120.0f, 58.0f};
        head->material.vector = {0.95f, 0.82f, 0.28f};
        head->material.density = 2.6f;
        head->material.hardness = 0.84f;
        head->material.elasticity = 0.18f;
        head->topology = ComponentTopology::PrimitiveSolid;
        head->dimensions = {head->size.x / kPixelsPerMeter, head->size.y / kPixelsPerMeter, 0.55f};

        grip->position = genesis::Vec2f{7.0f, 5.0f};
        shaft->position = genesis::Vec2f{10.0f, 5.2f};
        head->position = genesis::Vec2f{13.0f, 4.9f};

        sim.toolModel.AddComponent(grip);
        sim.toolModel.AddComponent(shaft);
        sim.toolModel.AddComponent(head);

        sim.parts.reserve(3);
        const std::array<b2Vec2, 3> initialPositions = {
            b2Vec2{7.0f, 5.0f},
            b2Vec2{10.0f, 5.2f},
            b2Vec2{13.0f, 4.9f}};

        const std::array<std::shared_ptr<Component>, 3> components = {grip, shaft, head};
        for (std::size_t i = 0; i < components.size(); ++i)
        {
            ToolPartRuntime part;
            part.component = components[i];
            part.tag = {BodyTag::Kind::ToolPart, i};
            part.displayDepth = static_cast<float>(i) * 0.18f;

            b2BodyDef bodyDef = b2DefaultBodyDef();
            bodyDef.type = b2_dynamicBody;
            bodyDef.position = initialPositions[i];
            bodyDef.linearDamping = 0.08f;
            bodyDef.angularDamping = 1.0f;
            part.bodyId = b2CreateBody(sim.worldId, &bodyDef);

            b2ShapeDef shapeDef = b2DefaultShapeDef();
            shapeDef.density = part.component->material.density;
            shapeDef.material.friction = 0.55f;
            shapeDef.material.restitution = 0.1f;
            shapeDef.enableContactEvents = true;
            shapeDef.enableHitEvents = true;

            const b2Polygon box = b2MakeBox(part.component->size.x / (2.0f * kPixelsPerMeter), part.component->size.y / (2.0f * kPixelsPerMeter));
            part.shapeId = b2CreatePolygonShape(part.bodyId, &shapeDef, &box);
            sim.parts.push_back(part);
            auto &storedPart = sim.parts.back();
            b2Body_SetUserData(storedPart.bodyId, &storedPart.tag);
            shapeDef.userData = &storedPart.tag;
            b2Shape_SetUserData(storedPart.shapeId, &storedPart.tag);
        }

        sim.resources.reserve(2);
        {
            ResourceRuntime resource;
            resource.node = genesis::ResourceNode("Pine Tree", 0.35f, 100.0f, GREEN, genesis::Vec2f{4.0f, 2.0f}, genesis::Vec2f{1.8f, 3.6f});
            resource.tag = {BodyTag::Kind::Resource, 0};
            resource.node.SetPhysicsBody(b2_nullBodyId);
            resource.displayDepth = -1.0f;

            b2BodyDef bodyDef = b2DefaultBodyDef();
            bodyDef.type = b2_staticBody;
            bodyDef.position = b2Vec2{4.0f, 2.0f};
            resource.bodyId = b2CreateBody(sim.worldId, &bodyDef);

            b2ShapeDef shapeDef = b2DefaultShapeDef();
            shapeDef.enableContactEvents = true;
            shapeDef.enableHitEvents = true;
            const b2Polygon box = b2MakeBox(resource.node.Size().x * 0.5f, resource.node.Size().y * 0.5f);
            resource.shapeId = b2CreatePolygonShape(resource.bodyId, &shapeDef, &box);
            resource.node.SetPhysicsBody(resource.bodyId);
            sim.resources.push_back(resource);
            auto &storedResource = sim.resources.back();
            b2Body_SetUserData(storedResource.bodyId, &storedResource.tag);
            b2Shape_SetUserData(storedResource.shapeId, &storedResource.tag);
        }
        {
            ResourceRuntime resource;
            resource.node = genesis::ResourceNode("Granite Boulder", 0.75f, 250.0f, LIGHTGRAY, genesis::Vec2f{16.0f, 1.4f}, genesis::Vec2f{2.2f, 1.5f});
            resource.tag = {BodyTag::Kind::Resource, 1};
            resource.node.SetPhysicsBody(b2_nullBodyId);
            resource.displayDepth = -0.8f;

            b2BodyDef bodyDef = b2DefaultBodyDef();
            bodyDef.type = b2_staticBody;
            bodyDef.position = b2Vec2{16.0f, 1.4f};
            resource.bodyId = b2CreateBody(sim.worldId, &bodyDef);

            b2ShapeDef shapeDef = b2DefaultShapeDef();
            shapeDef.enableContactEvents = true;
            shapeDef.enableHitEvents = true;
            const b2Polygon box = b2MakeBox(resource.node.Size().x * 0.5f, resource.node.Size().y * 0.5f);
            resource.shapeId = b2CreatePolygonShape(resource.bodyId, &shapeDef, &box);
            resource.node.SetPhysicsBody(resource.bodyId);
            sim.resources.push_back(resource);
            auto &storedResource = sim.resources.back();
            b2Body_SetUserData(storedResource.bodyId, &storedResource.tag);
            b2Shape_SetUserData(storedResource.shapeId, &storedResource.tag);
        }

        b2BodyDef floorDef = b2DefaultBodyDef();
        floorDef.position = b2Vec2{10.0f, -0.25f};
        sim.floorId = b2CreateBody(sim.worldId, &floorDef);
        {
            b2ShapeDef shapeDef = b2DefaultShapeDef();
            b2Polygon floorShape = b2MakeBox(20.0f, 0.25f);
            b2CreatePolygonShape(sim.floorId, &shapeDef, &floorShape);
        }

        b2BodyDef leftWallDef = b2DefaultBodyDef();
        leftWallDef.position = b2Vec2{-0.25f, 6.0f};
        sim.leftWallId = b2CreateBody(sim.worldId, &leftWallDef);
        {
            b2ShapeDef shapeDef = b2DefaultShapeDef();
            b2Polygon wallShape = b2MakeBox(0.25f, 6.0f);
            b2CreatePolygonShape(sim.leftWallId, &shapeDef, &wallShape);
        }

        b2BodyDef rightWallDef = b2DefaultBodyDef();
        rightWallDef.position = b2Vec2{20.25f, 6.0f};
        sim.rightWallId = b2CreateBody(sim.worldId, &rightWallDef);
        {
            b2ShapeDef shapeDef = b2DefaultShapeDef();
            b2Polygon wallShape = b2MakeBox(0.25f, 6.0f);
            b2CreatePolygonShape(sim.rightWallId, &shapeDef, &wallShape);
        }

        b2World_SetGravity(sim.worldId, b2Vec2{0.0f, -18.0f});
        b2World_SetHitEventThreshold(sim.worldId, 1.0f);
        b2World_EnableSleeping(sim.worldId, false);
    }

} // namespace
