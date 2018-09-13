// The MIT License(MIT)
//
// Copyright(c) 2016 Cedric Guillemet
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files(the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and / or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions :
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <imgui/imgui.h>
#ifndef IMGUI_DEFINE_MATH_OPERATORS
#define IMGUI_DEFINE_MATH_OPERATORS
#endif
#include <imgui/imgui_internal.h>
#include "ImGuizmo.h"

// includes patches for multiview from
// https://github.com/CedricGuillemet/ImGuizmo/issues/15

#include "ImGuizmo_internal.h"

namespace ImGuizmo
{
   static const float ZPI = 3.14159265358979323846f;
   static const float RAD2DEG = (180.f / ZPI);
   static const float DEG2RAD = (ZPI / 180.f);
   static const float gGizmoSizeClipSpace = 0.1f;
   const float screenRotateSize = 0.06f;

   //static Context gContext;

   static const float angleLimit = 0.96f;
   static const float planeLimit = 0.2f;

   static const vec_t directionUnary[3] = { makeVect(1.f, 0.f, 0.f), makeVect(0.f, 1.f, 0.f), makeVect(0.f, 0.f, 1.f) };
   static const ImU32 directionColor[3] = { 0xFF0000AA, 0xFF00AA00, 0xFFAA0000 };

   // Alpha: 100%: FF, 87%: DE, 70%: B3, 54%: 8A, 50%: 80, 38%: 61, 12%: 1F
   static const ImU32 planeColor[3] = { 0x610000AA, 0x6100AA00, 0x61AA0000 };
   static const ImU32 selectionColor = 0x8A1080FF;
   static const ImU32 inactiveColor = 0x99999999;
   static const ImU32 translationLineColor = 0xAAAAAAAA;
   static const char *translationInfoMask[] = { "X : %5.3f", "Y : %5.3f", "Z : %5.3f",
      "Y : %5.3f Z : %5.3f", "X : %5.3f Z : %5.3f", "X : %5.3f Y : %5.3f",
      "X : %5.3f Y : %5.3f Z : %5.3f" };
   static const char *scaleInfoMask[] = { "X : %5.2f", "Y : %5.2f", "Z : %5.2f", "XYZ : %5.2f" };
   static const char *rotationInfoMask[] = { "X : %5.2f deg %5.2f rad", "Y : %5.2f deg %5.2f rad", "Z : %5.2f deg %5.2f rad", "Screen : %5.2f deg %5.2f rad" };
   static const int translationInfoIndex[] = { 0,0,0, 1,0,0, 2,0,0, 1,2,0, 0,2,0, 0,1,0, 0,1,2 };
   static const float quadMin = 0.5f;
   static const float quadMax = 0.8f;
   static const float quadUV[8] = { quadMin, quadMin, quadMin, quadMax, quadMax, quadMax, quadMax, quadMin };
   static const int halfCircleSegmentCount = 64;
   static const float snapTension = 0.5f;

   ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   static Context * gContext = NULL;

   void * CreateContext(void)
   {
      return new Context();
   }

   void DestroyContext(void * ctx)
   {
      Context * c = reinterpret_cast<Context *>(ctx);
      delete c;
   }

   void * GetCurrentContext(void)
   {
      return reinterpret_cast<void *>(gContext);
   }

   void SetCurrentContext(void * ctx)
   {
      gContext = reinterpret_cast<Context *>(ctx);
   }
   ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

   ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   //
   static int GetMoveType(vec_t *gizmoHitProportion);
   static int GetRotateType();
   static int GetScaleType();

   static ImVec2 worldToPos(const vec_t& worldPos, const matrix_t& mat)
   {
      vec_t trans;
      trans.TransformPoint(worldPos, mat);
      trans *= 0.5f / trans.w;
      trans += makeVect(0.5f, 0.5f);
      trans.y = 1.f - trans.y;
      trans.x *= gContext->mWidth;
      trans.y *= gContext->mHeight;
      trans.x += gContext->mX;
      trans.y += gContext->mY;
      return ImVec2(trans.x, trans.y);
   }

   static void ComputeCameraRay(vec_t &rayOrigin, vec_t &rayDir)
   {
      ImGuiIO& io = ImGui::GetIO();

      matrix_t mViewProjInverse;
      mViewProjInverse.Inverse(gContext->mViewMat * gContext->mProjectionMat);

      float mox = ((io.MousePos.x - gContext->mX) / gContext->mWidth) * 2.f - 1.f;
      float moy = (1.f - ((io.MousePos.y - gContext->mY) / gContext->mHeight)) * 2.f - 1.f;

      rayOrigin.Transform(makeVect(mox, moy, 0.f, 1.f), mViewProjInverse);
      rayOrigin *= 1.f / rayOrigin.w;
      vec_t rayEnd;
      rayEnd.Transform(makeVect(mox, moy, 1.f, 1.f), mViewProjInverse);
      rayEnd *= 1.f / rayEnd.w;
      rayDir = Normalized(rayEnd - rayOrigin);
   }

   static float GetSegmentLengthClipSpace(const vec_t& start, const vec_t& end)
   {
      vec_t startOfSegment = start;
      startOfSegment.TransformPoint(gContext->mMVP);
      if (fabsf(startOfSegment.w)> FLT_EPSILON) // check for axis aligned with camera direction
         startOfSegment *= 1.f / startOfSegment.w;

      vec_t endOfSegment = end;
      endOfSegment.TransformPoint(gContext->mMVP);
      if (fabsf(endOfSegment.w)> FLT_EPSILON) // check for axis aligned with camera direction
         endOfSegment *= 1.f / endOfSegment.w;

      vec_t clipSpaceAxis = endOfSegment - startOfSegment;
      clipSpaceAxis.y /= gContext->mDisplayRatio;
      float segmentLengthInClipSpace = sqrtf(clipSpaceAxis.x*clipSpaceAxis.x + clipSpaceAxis.y*clipSpaceAxis.y);
      return segmentLengthInClipSpace;
   }

   static float GetParallelogram(const vec_t& ptO, const vec_t& ptA, const vec_t& ptB)
   {
      vec_t pts[] = { ptO, ptA, ptB };
      for (unsigned int i = 0; i < 3; i++)
      {
         pts[i].TransformPoint(gContext->mMVP);
         if (fabsf(pts[i].w)> FLT_EPSILON) // check for axis aligned with camera direction
            pts[i] *= 1.f / pts[i].w;
      }
      vec_t segA = pts[1] - pts[0];
      vec_t segB = pts[2] - pts[0];
      segA.y /= gContext->mDisplayRatio;
      segB.y /= gContext->mDisplayRatio;
      vec_t segAOrtho = makeVect(-segA.y, segA.x);
      segAOrtho.Normalize();
      float dt = segAOrtho.Dot3(segB);
      float surface = sqrtf(segA.x*segA.x + segA.y*segA.y) * fabsf(dt);
      return surface;
   }

   inline vec_t PointOnSegment(const vec_t & point, const vec_t & vertPos1, const vec_t & vertPos2)
   {
      vec_t c = point - vertPos1;
      vec_t V;

      V.Normalize(vertPos2 - vertPos1);
      float d = (vertPos2 - vertPos1).Length();
      float t = V.Dot3(c);

      if (t < 0.f)
         return vertPos1;

      if (t > d)
         return vertPos2;

      return vertPos1 + V * t;
   }

   static float IntersectRayPlane(const vec_t & rOrigin, const vec_t& rVector, const vec_t& plan)
   {
      float numer = plan.Dot3(rOrigin) - plan.w;
      float denom = plan.Dot3(rVector);

      if (fabsf(denom) < FLT_EPSILON)  // normal is orthogonal to vector, cant intersect
         return -1.0f;

      return -(numer / denom);
   }

   static bool IsInContextRect( ImVec2 p )
   {
       return IsWithin( p.x, gContext->mX, gContext->mXMax ) && IsWithin(p.y, gContext->mY, gContext->mYMax );
   }

   void SetRect(float x, float y, float width, float height)
   {
       gContext->mX = x;
       gContext->mY = y;
       gContext->mWidth = width;
       gContext->mHeight = height;
       gContext->mXMax = gContext->mX + gContext->mWidth;
       gContext->mYMax = gContext->mY + gContext->mXMax;
      gContext->mDisplayRatio = width / height;
   }

   IMGUI_API void SetOrthographic(bool isOrthographic)
   {
      gContext->mIsOrthographic = isOrthographic;
   }

   void SetDrawlist()
   {
      gContext->mDrawList = ImGui::GetWindowDrawList();
   }

   void BeginFrame()
   {
      ImGuiIO& io = ImGui::GetIO();

      const ImU32 flags = ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoInputs | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoBringToFrontOnFocus;
      ImGui::SetNextWindowSize(io.DisplaySize);
      ImGui::SetNextWindowPos(ImVec2(0, 0));
      
      ImGui::PushStyleColor(ImGuiCol_WindowBg, 0);
      ImGui::PushStyleColor(ImGuiCol_Border, 0);
      ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
     
      ImGui::Begin("gizmo", NULL, flags);
      gContext->mDrawList = ImGui::GetWindowDrawList();
      ImGui::End();
      ImGui::PopStyleVar();
      ImGui::PopStyleColor(2);
   }

   bool IsUsing()
   {
      return gContext->mbUsing||gContext->mbUsingBounds;
   }

   bool IsOver()
   {
      return (GetMoveType(NULL) != NONE) || GetRotateType() != NONE || GetScaleType() != NONE || IsUsing();
   }

   void Enable(bool enable)
   {
      gContext->mbEnable = enable;
      if (!enable)
      {
          gContext->mbUsing = false;
          gContext->mbUsingBounds = false;
      }
   }

   static float GetUniform(const vec_t& position, const matrix_t& mat)
   {
      vec_t trf = makeVect(position.x, position.y, position.z, 1.f);
      trf.Transform(mat);
      return trf.w;
   }

   static void ComputeContext(const float *view, const float *projection, float *matrix, MODE mode)
   {
      gContext->mMode = mode;
      gContext->mViewMat = *(matrix_t*)view;
      gContext->mProjectionMat = *(matrix_t*)projection;

      if (mode == LOCAL)
      {
         gContext->mModel = *(matrix_t*)matrix;
         gContext->mModel.OrthoNormalize();
      }
      else
      {
         gContext->mModel.Translation(((matrix_t*)matrix)->v.position);
      }
      gContext->mModelSource = *(matrix_t*)matrix;
      gContext->mModelScaleOrigin.Set(gContext->mModelSource.v.right.Length(), gContext->mModelSource.v.up.Length(), gContext->mModelSource.v.dir.Length());

      gContext->mModelInverse.Inverse(gContext->mModel);
      gContext->mModelSourceInverse.Inverse(gContext->mModelSource);
      gContext->mViewProjection = gContext->mViewMat * gContext->mProjectionMat;
      gContext->mMVP = gContext->mModel * gContext->mViewProjection;

      matrix_t viewInverse;
      viewInverse.Inverse(gContext->mViewMat);
      gContext->mCameraDir = viewInverse.v.dir;
      gContext->mCameraEye = viewInverse.v.position;
      gContext->mCameraRight = viewInverse.v.right;
      gContext->mCameraUp = viewInverse.v.up;

     // compute scale from the size of camera right vector projected on screen at the matrix position
     vec_t pointRight = viewInverse.v.right;
     pointRight.TransformPoint(gContext->mViewProjection);
     gContext->mScreenFactor = gGizmoSizeClipSpace / (pointRight.x / pointRight.w - gContext->mMVP.v.position.x / gContext->mMVP.v.position.w);

     vec_t rightViewInverse = viewInverse.v.right;
     rightViewInverse.TransformVector(gContext->mModelInverse);
     float rightLength = GetSegmentLengthClipSpace(makeVect(0.f, 0.f), rightViewInverse);
     gContext->mScreenFactor = gGizmoSizeClipSpace / rightLength;

      ImVec2 centerSSpace = worldToPos(makeVect(0.f, 0.f), gContext->mMVP);
      gContext->mScreenSquareCenter = centerSSpace;
      gContext->mScreenSquareMin = ImVec2(centerSSpace.x - 10.f, centerSSpace.y - 10.f);
      gContext->mScreenSquareMax = ImVec2(centerSSpace.x + 10.f, centerSSpace.y + 10.f);

      ComputeCameraRay(gContext->mRayOrigin, gContext->mRayVector);
   }

   static void ComputeColors(ImU32 *colors, int type, OPERATION operation)
   {
      if (gContext->mbEnable)
      {
         switch (operation)
         {
         case TRANSLATE:
            colors[0] = (type == MOVE_SCREEN) ? selectionColor : 0xFFFFFFFF;
            for (int i = 0; i < 3; i++)
            {
               colors[i + 1] = (type == (int)(MOVE_X + i)) ? selectionColor : directionColor[i];
               colors[i + 4] = (type == (int)(MOVE_YZ + i)) ? selectionColor : planeColor[i];
               colors[i + 4] = (type == MOVE_SCREEN) ? selectionColor : colors[i + 4];
            }
            break;
         case ROTATE:
            colors[0] = (type == ROTATE_SCREEN) ? selectionColor : 0xFFFFFFFF;
            for (int i = 0; i < 3; i++)
               colors[i + 1] = (type == (int)(ROTATE_X + i)) ? selectionColor : directionColor[i];
            break;
         case SCALE:
            colors[0] = (type == SCALE_XYZ) ? selectionColor : 0xFFFFFFFF;
            for (int i = 0; i < 3; i++)
               colors[i + 1] = (type == (int)(SCALE_X + i)) ? selectionColor : directionColor[i];
            break;
         case BOUNDS:
            break;
         }
      }
      else
      {
         for (int i = 0; i < 7; i++)
            colors[i] = inactiveColor;
      }
   }

   static void ComputeTripodAxisAndVisibility(int axisIndex, vec_t& dirAxis, vec_t& dirPlaneX, vec_t& dirPlaneY, bool& belowAxisLimit, bool& belowPlaneLimit)
   {
      dirAxis = directionUnary[axisIndex];
      dirPlaneX = directionUnary[(axisIndex + 1) % 3];
      dirPlaneY = directionUnary[(axisIndex + 2) % 3];

      if (gContext->mbUsing)
      {
         // when using, use stored factors so the gizmo doesn't flip when we translate
         belowAxisLimit = gContext->mBelowAxisLimit[axisIndex];
         belowPlaneLimit = gContext->mBelowPlaneLimit[axisIndex];

         dirAxis *= gContext->mAxisFactor[axisIndex];
         dirPlaneX *= gContext->mAxisFactor[(axisIndex + 1) % 3];
         dirPlaneY *= gContext->mAxisFactor[(axisIndex + 2) % 3];
      }
      else
      {
         // new method
         float lenDir = GetSegmentLengthClipSpace(makeVect(0.f, 0.f, 0.f), dirAxis);
         float lenDirMinus = GetSegmentLengthClipSpace(makeVect(0.f, 0.f, 0.f), -dirAxis);

         float lenDirPlaneX = GetSegmentLengthClipSpace(makeVect(0.f, 0.f, 0.f), dirPlaneX);
         float lenDirMinusPlaneX = GetSegmentLengthClipSpace(makeVect(0.f, 0.f, 0.f), -dirPlaneX);

         float lenDirPlaneY = GetSegmentLengthClipSpace(makeVect(0.f, 0.f, 0.f), dirPlaneY);
         float lenDirMinusPlaneY = GetSegmentLengthClipSpace(makeVect(0.f, 0.f, 0.f), -dirPlaneY);

         float mulAxis = (lenDir < lenDirMinus && fabsf(lenDir - lenDirMinus) > FLT_EPSILON) ? -1.f : 1.f;
         float mulAxisX = (lenDirPlaneX < lenDirMinusPlaneX && fabsf(lenDirPlaneX - lenDirMinusPlaneX) > FLT_EPSILON) ? -1.f : 1.f;
         float mulAxisY = (lenDirPlaneY < lenDirMinusPlaneY && fabsf(lenDirPlaneY - lenDirMinusPlaneY) > FLT_EPSILON) ? -1.f : 1.f;
         dirAxis *= mulAxis;
         dirPlaneX *= mulAxisX;
         dirPlaneY *= mulAxisY;

         // for axis
         float axisLengthInClipSpace = GetSegmentLengthClipSpace(makeVect(0.f, 0.f, 0.f), dirAxis * gContext->mScreenFactor);

         float paraSurf = GetParallelogram(makeVect(0.f, 0.f, 0.f), dirPlaneX * gContext->mScreenFactor, dirPlaneY * gContext->mScreenFactor);
         belowPlaneLimit = (paraSurf > 0.0025f);
         belowAxisLimit = (axisLengthInClipSpace > 0.02f);

         // and store values
         gContext->mAxisFactor[axisIndex] = mulAxis;
         gContext->mAxisFactor[(axisIndex + 1) % 3] = mulAxisX;
         gContext->mAxisFactor[(axisIndex + 2) % 3] = mulAxisY;
         gContext->mBelowAxisLimit[axisIndex] = belowAxisLimit;
         gContext->mBelowPlaneLimit[axisIndex] = belowPlaneLimit;
      }
   }

   static void ComputeSnap(float*value, float snap)
   {
      if (snap <= FLT_EPSILON)
         return;
      float modulo = fmodf(*value, snap);
      float moduloRatio = fabsf(modulo) / snap;
      if (moduloRatio < snapTension)
         *value -= modulo;
      else if (moduloRatio >(1.f - snapTension))
         *value = *value - modulo + snap * ((*value<0.f) ? -1.f : 1.f);
   }
   static void ComputeSnap(vec_t& value, float *snap)
   {
      for (int i = 0; i < 3; i++)
      {
         ComputeSnap(&value[i], snap[i]);
      }
   }

   static float ComputeAngleOnPlan()
   {
      const float len = IntersectRayPlane(gContext->mRayOrigin, gContext->mRayVector, gContext->mTranslationPlan);
      vec_t localPos = Normalized(gContext->mRayOrigin + gContext->mRayVector * len - gContext->mModel.v.position);

      vec_t perpendicularVector;
      perpendicularVector.Cross(gContext->mRotationVectorSource, gContext->mTranslationPlan);
      perpendicularVector.Normalize();
      float acosAngle = Clamp(Dot(localPos, gContext->mRotationVectorSource), -0.9999f, 0.9999f);
      float angle = acosf(acosAngle);
      angle *= (Dot(localPos, perpendicularVector) < 0.f) ? 1.f : -1.f;
      return angle;
   }

   static void DrawRotationGizmo(int type)
   {
      ImDrawList* drawList = gContext->mDrawList;

      // colors
      ImU32 colors[7];
      ComputeColors(colors, type, ROTATE);

     vec_t cameraToModelNormalized;
     if (gContext->mIsOrthographic)
     {
        matrix_t viewInverse;
        viewInverse.Inverse(*(matrix_t*)&gContext->mViewMat);
        cameraToModelNormalized = viewInverse.v.dir;
     }
     else
     {
        cameraToModelNormalized = Normalized(gContext->mModel.v.position - gContext->mCameraEye);
     }

      cameraToModelNormalized.TransformVector(gContext->mModelInverse);

      gContext->mRadiusSquareCenter = screenRotateSize * gContext->mHeight;

     for (int axis = 0; axis < 3; axis++)
      {
         ImVec2 circlePos[halfCircleSegmentCount];

         float angleStart = atan2f(cameraToModelNormalized[(4-axis)%3], cameraToModelNormalized[(3 - axis) % 3]) + ZPI * 0.5f;

         for (unsigned int i = 0; i < halfCircleSegmentCount; i++)
         {
            float ng = angleStart + ZPI * ((float)i / (float)halfCircleSegmentCount);
            vec_t axisPos = makeVect(cosf(ng), sinf(ng), 0.f);
            vec_t pos = makeVect(axisPos[axis], axisPos[(axis+1)%3], axisPos[(axis+2)%3]) * gContext->mScreenFactor;
            circlePos[i] = worldToPos(pos, gContext->mMVP);
         }

         float radiusAxis = sqrtf( (ImLengthSqr(worldToPos(gContext->mModel.v.position, gContext->mViewProjection) - circlePos[0]) ));
         if(radiusAxis > gContext->mRadiusSquareCenter)
           gContext->mRadiusSquareCenter = radiusAxis;

         drawList->AddPolyline(circlePos, halfCircleSegmentCount, colors[3 - axis], false, 2);
      }
      drawList->AddCircle(worldToPos(gContext->mModel.v.position, gContext->mViewProjection), gContext->mRadiusSquareCenter, colors[0], 64, 3.f);

      if (gContext->mbUsing)
      {
         ImVec2 circlePos[halfCircleSegmentCount +1];

         circlePos[0] = worldToPos(gContext->mModel.v.position, gContext->mViewProjection);
         for (unsigned int i = 1; i < halfCircleSegmentCount; i++)
         {
            float ng = gContext->mRotationAngle * ((float)(i-1) / (float)(halfCircleSegmentCount -1));
            matrix_t rotateVectorMatrix;
            rotateVectorMatrix.RotationAxis(gContext->mTranslationPlan, ng);
            vec_t pos;
            pos.TransformPoint(gContext->mRotationVectorSource, rotateVectorMatrix);
            pos *= gContext->mScreenFactor;
            circlePos[i] = worldToPos(pos + gContext->mModel.v.position, gContext->mViewProjection);
         }
         drawList->AddConvexPolyFilled(circlePos, halfCircleSegmentCount, 0x801080FF);
         drawList->AddPolyline(circlePos, halfCircleSegmentCount, 0xFF1080FF, true, 2);

         ImVec2 destinationPosOnScreen = circlePos[1];
         char tmps[512];
         ImFormatString(tmps, sizeof(tmps), rotationInfoMask[type - ROTATE_X], (gContext->mRotationAngle/ZPI)*180.f, gContext->mRotationAngle);
         drawList->AddText(ImVec2(destinationPosOnScreen.x + 15, destinationPosOnScreen.y + 15), 0xFF000000, tmps);
         drawList->AddText(ImVec2(destinationPosOnScreen.x + 14, destinationPosOnScreen.y + 14), 0xFFFFFFFF, tmps);
      }
   }

   static void DrawHatchedAxis(const vec_t& axis)
   {
      for (int j = 1; j < 10; j++)
      {
         ImVec2 baseSSpace2 = worldToPos(axis * 0.05f * (float)(j * 2) * gContext->mScreenFactor, gContext->mMVP);
         ImVec2 worldDirSSpace2 = worldToPos(axis * 0.05f * (float)(j * 2 + 1) * gContext->mScreenFactor, gContext->mMVP);
         gContext->mDrawList->AddLine(baseSSpace2, worldDirSSpace2, 0x80000000, 6.f);
      }
   }

   static void DrawScaleGizmo(int type)
   {
      ImDrawList* drawList = gContext->mDrawList;

      // colors
      ImU32 colors[7];
      ComputeColors(colors, type, SCALE);

      // draw
      vec_t scaleDisplay = { 1.f, 1.f, 1.f, 1.f };

      if (gContext->mbUsing)
         scaleDisplay = gContext->mScale;

      for (unsigned int i = 0; i < 3; i++)
      {
        vec_t dirPlaneX, dirPlaneY, dirAxis;
         bool belowAxisLimit, belowPlaneLimit;
         ComputeTripodAxisAndVisibility(i, dirAxis, dirPlaneX, dirPlaneY, belowAxisLimit, belowPlaneLimit);

         // draw axis
         if (belowAxisLimit)
         {
            ImVec2 baseSSpace = worldToPos(dirAxis * 0.1f * gContext->mScreenFactor, gContext->mMVP);
            ImVec2 worldDirSSpaceNoScale = worldToPos(dirAxis * gContext->mScreenFactor, gContext->mMVP);
            ImVec2 worldDirSSpace = worldToPos((dirAxis * scaleDisplay[i]) * gContext->mScreenFactor, gContext->mMVP);

            if (gContext->mbUsing)
            {
               drawList->AddLine(baseSSpace, worldDirSSpaceNoScale, 0xFF404040, 3.f);
               drawList->AddCircleFilled(worldDirSSpaceNoScale, 6.f, 0xFF404040);
            }

            drawList->AddLine(baseSSpace, worldDirSSpace, colors[i + 1], 3.f);
            drawList->AddCircleFilled(worldDirSSpace, 6.f, colors[i + 1]);

            if (gContext->mAxisFactor[i] < 0.f)
               DrawHatchedAxis(dirAxis * scaleDisplay[i]);
         }
      }

      // draw screen cirle
      drawList->AddCircleFilled(gContext->mScreenSquareCenter, 6.f, colors[0], 32);

      if (gContext->mbUsing)
      {
         //ImVec2 sourcePosOnScreen = worldToPos(gContext->mMatrixOrigin, gContext->mViewProjection);
         ImVec2 destinationPosOnScreen = worldToPos(gContext->mModel.v.position, gContext->mViewProjection);
         /*vec_t dif(destinationPosOnScreen.x - sourcePosOnScreen.x, destinationPosOnScreen.y - sourcePosOnScreen.y);
         dif.Normalize();
         dif *= 5.f;
         drawList->AddCircle(sourcePosOnScreen, 6.f, translationLineColor);
         drawList->AddCircle(destinationPosOnScreen, 6.f, translationLineColor);
         drawList->AddLine(ImVec2(sourcePosOnScreen.x + dif.x, sourcePosOnScreen.y + dif.y), ImVec2(destinationPosOnScreen.x - dif.x, destinationPosOnScreen.y - dif.y), translationLineColor, 2.f);
         */
         char tmps[512];
         //vec_t deltaInfo = gContext->mModel.v.position - gContext->mMatrixOrigin;
         int componentInfoIndex = (type - SCALE_X) * 3;
         ImFormatString(tmps, sizeof(tmps), scaleInfoMask[type - SCALE_X], scaleDisplay[translationInfoIndex[componentInfoIndex]]);
         drawList->AddText(ImVec2(destinationPosOnScreen.x + 15, destinationPosOnScreen.y + 15), 0xFF000000, tmps);
         drawList->AddText(ImVec2(destinationPosOnScreen.x + 14, destinationPosOnScreen.y + 14), 0xFFFFFFFF, tmps);
      }
   }


   static void DrawTranslationGizmo(int type)
   {
      ImDrawList* drawList = gContext->mDrawList;
      if (!drawList)
          return;

      // colors
      ImU32 colors[7];
      ComputeColors(colors, type, TRANSLATE);

      const ImVec2 origin = worldToPos(gContext->mModel.v.position, gContext->mViewProjection);

      // draw
      bool belowAxisLimit = false;
      bool belowPlaneLimit = false;
      for (unsigned int i = 0; i < 3; ++i)
      {
         vec_t dirPlaneX, dirPlaneY, dirAxis;
         ComputeTripodAxisAndVisibility(i, dirAxis, dirPlaneX, dirPlaneY, belowAxisLimit, belowPlaneLimit);

         // draw axis
         if (belowAxisLimit)
         {
            ImVec2 baseSSpace = worldToPos(dirAxis * 0.1f * gContext->mScreenFactor, gContext->mMVP);
            ImVec2 worldDirSSpace = worldToPos(dirAxis * gContext->mScreenFactor, gContext->mMVP);

            drawList->AddLine(baseSSpace, worldDirSSpace, colors[i + 1], 3.f);

            // Arrow head begin
            ImVec2 dir(origin - worldDirSSpace);

            float d = sqrtf(ImLengthSqr(dir));
            dir /= d; // Normalize
            dir *= 6.0f;

            ImVec2 ortogonalDir(dir.y, -dir.x); // Perpendicular vector
            ImVec2 a(worldDirSSpace + dir);
            drawList->AddTriangleFilled(worldDirSSpace - dir, a + ortogonalDir, a - ortogonalDir, colors[i + 1]);
            // Arrow head end

            if (gContext->mAxisFactor[i] < 0.f)
               DrawHatchedAxis(dirAxis);
         }

         // draw plane
         if (belowPlaneLimit)
         {
            ImVec2 screenQuadPts[4];
            for (int j = 0; j < 4; ++j)
            {
               vec_t cornerWorldPos = (dirPlaneX * quadUV[j * 2] + dirPlaneY  * quadUV[j * 2 + 1]) * gContext->mScreenFactor;
               screenQuadPts[j] = worldToPos(cornerWorldPos, gContext->mMVP);
            }
            drawList->AddPolyline(screenQuadPts, 4, directionColor[i], true, 1.0f);
            drawList->AddConvexPolyFilled(screenQuadPts, 4, colors[i + 4]);
         }
      }

      drawList->AddCircleFilled(gContext->mScreenSquareCenter, 6.f, colors[0], 32);

      if (gContext->mbUsing)
      {
         ImVec2 sourcePosOnScreen = worldToPos(gContext->mMatrixOrigin, gContext->mViewProjection);
         ImVec2 destinationPosOnScreen = worldToPos(gContext->mModel.v.position, gContext->mViewProjection);
         vec_t dif = { destinationPosOnScreen.x - sourcePosOnScreen.x, destinationPosOnScreen.y - sourcePosOnScreen.y, 0.f, 0.f };
         dif.Normalize();
         dif *= 5.f;
         drawList->AddCircle(sourcePosOnScreen, 6.f, translationLineColor);
         drawList->AddCircle(destinationPosOnScreen, 6.f, translationLineColor);
         drawList->AddLine(ImVec2(sourcePosOnScreen.x + dif.x, sourcePosOnScreen.y + dif.y), ImVec2(destinationPosOnScreen.x - dif.x, destinationPosOnScreen.y - dif.y), translationLineColor, 2.f);

         char tmps[512];
         vec_t deltaInfo = gContext->mModel.v.position - gContext->mMatrixOrigin;
         int componentInfoIndex = (type - MOVE_X) * 3;
         ImFormatString(tmps, sizeof(tmps), translationInfoMask[type - MOVE_X], deltaInfo[translationInfoIndex[componentInfoIndex]], deltaInfo[translationInfoIndex[componentInfoIndex + 1]], deltaInfo[translationInfoIndex[componentInfoIndex + 2]]);
         drawList->AddText(ImVec2(destinationPosOnScreen.x + 15, destinationPosOnScreen.y + 15), 0xFF000000, tmps);
         drawList->AddText(ImVec2(destinationPosOnScreen.x + 14, destinationPosOnScreen.y + 14), 0xFFFFFFFF, tmps);
      }
   }

   static bool CanActivate()
   {
      if (ImGui::IsMouseClicked(0) && !ImGui::IsAnyItemHovered() && !ImGui::IsAnyItemActive())
         return true;
      return false;
   }

   static void HandleAndDrawLocalBounds(float *bounds, matrix_t *matrix, float *snapValues, OPERATION operation)
   {
       ImGuiIO& io = ImGui::GetIO();
       ImDrawList* drawList = gContext->mDrawList;

       // compute best projection axis
       vec_t axesWorldDirections[3];
       vec_t bestAxisWorldDirection = { 0.0f, 0.0f, 0.0f, 0.0f };
       int axes[3];
       unsigned int numAxes = 1;
       axes[0] = gContext->mBoundsBestAxis;
       int bestAxis = axes[0];
       if (!gContext->mbUsingBounds)
       {
           numAxes = 0;
           float bestDot = 0.f;
           for (unsigned int i = 0; i < 3; i++)
           {
               vec_t dirPlaneNormalWorld;
               dirPlaneNormalWorld.TransformVector(directionUnary[i], gContext->mModelSource);
               dirPlaneNormalWorld.Normalize();

               float dt = fabsf( Dot(Normalized(gContext->mCameraEye - gContext->mModelSource.v.position), dirPlaneNormalWorld) );
               if ( dt >= bestDot )
               {
                   bestDot = dt;
                   bestAxis = i;
                   bestAxisWorldDirection = dirPlaneNormalWorld;
               }

               if( dt >= 0.1f )
               {
                   axes[numAxes] = i;
                   axesWorldDirections[numAxes] = dirPlaneNormalWorld;
                   ++numAxes;
               }
           }
       }

       if( numAxes == 0 )
       {
            axes[0] = bestAxis;
            axesWorldDirections[0] = bestAxisWorldDirection;
            numAxes = 1;
       }
       else if( bestAxis != axes[0] )
       {
          unsigned int bestIndex = 0;
          for (unsigned int i = 0; i < numAxes; i++)
          {
              if( axes[i] == bestAxis )
              {
                  bestIndex = i;
                  break;
              }
          }
          int tempAxis = axes[0];
          axes[0] = axes[bestIndex];
          axes[bestIndex] = tempAxis;
          vec_t tempDirection = axesWorldDirections[0];
          axesWorldDirections[0] = axesWorldDirections[bestIndex];
          axesWorldDirections[bestIndex] = tempDirection;
       }

       for (unsigned int axisIndex = 0; axisIndex < numAxes; ++axisIndex)
       {
           bestAxis = axes[axisIndex];
           bestAxisWorldDirection = axesWorldDirections[axisIndex];

           // corners
           vec_t aabb[4];

           int secondAxis = (bestAxis + 1) % 3;
           int thirdAxis = (bestAxis + 2) % 3;

           for (int i = 0; i < 4; i++)
           {
               aabb[i][3] = aabb[i][bestAxis] = 0.f;
               aabb[i][secondAxis] = bounds[secondAxis + 3 * (i >> 1)];
               aabb[i][thirdAxis] = bounds[thirdAxis + 3 * ((i >> 1) ^ (i & 1))];
           }

           // draw bounds
           unsigned int anchorAlpha = gContext->mbEnable ? 0xFF000000 : 0x80000000;

           matrix_t boundsMVP = gContext->mModelSource * gContext->mViewProjection;
           for (int i = 0; i < 4;i++)
           {
               ImVec2 worldBound1 = worldToPos(aabb[i], boundsMVP);
               ImVec2 worldBound2 = worldToPos(aabb[(i+1)%4], boundsMVP);
               if( !IsInContextRect( worldBound1 ) || !IsInContextRect( worldBound2 ) )
               {
                   continue;
               }
               float boundDistance = sqrtf(ImLengthSqr(worldBound1 - worldBound2));
               int stepCount = (int)(boundDistance / 10.f);
               stepCount = min( stepCount, 1000 );
               float stepLength = 1.f / (float)stepCount;
               for (int j = 0; j < stepCount; j++)
               {
                   float t1 = (float)j * stepLength;
                   float t2 = (float)j * stepLength + stepLength * 0.5f;
                   ImVec2 worldBoundSS1 = ImLerp(worldBound1, worldBound2, ImVec2(t1, t1));
                   ImVec2 worldBoundSS2 = ImLerp(worldBound1, worldBound2, ImVec2(t2, t2));
                   //drawList->AddLine(worldBoundSS1, worldBoundSS2, 0x000000 + anchorAlpha, 3.f);
               drawList->AddLine(worldBoundSS1, worldBoundSS2, 0xAAAAAA + anchorAlpha, 2.f);
               }
               vec_t midPoint = (aabb[i] + aabb[(i + 1) % 4] ) * 0.5f;
               ImVec2 midBound = worldToPos(midPoint, boundsMVP);
               static const float AnchorBigRadius = 8.f;
               static const float AnchorSmallRadius = 6.f;
               bool overBigAnchor = ImLengthSqr(worldBound1 - io.MousePos) <= (AnchorBigRadius*AnchorBigRadius);
               bool overSmallAnchor = ImLengthSqr(midBound - io.MousePos) <= (AnchorBigRadius*AnchorBigRadius);

            int type = NONE;
            vec_t gizmoHitProportion;

            switch (operation)
            {
            case TRANSLATE: type = GetMoveType(&gizmoHitProportion); break;
            case ROTATE: type = GetRotateType(); break;
            case SCALE: type = GetScaleType(); break;
            case BOUNDS: break;
            }
            if (type != NONE)
            {
               overBigAnchor = false;
               overSmallAnchor = false;
            }


               unsigned int bigAnchorColor = overBigAnchor ? selectionColor : (0xAAAAAA + anchorAlpha);
               unsigned int smallAnchorColor = overSmallAnchor ? selectionColor : (0xAAAAAA + anchorAlpha);

               drawList->AddCircleFilled(worldBound1, AnchorBigRadius, 0xFF000000);
            drawList->AddCircleFilled(worldBound1, AnchorBigRadius-1.2f, bigAnchorColor);

               drawList->AddCircleFilled(midBound, AnchorSmallRadius, 0xFF000000);
            drawList->AddCircleFilled(midBound, AnchorSmallRadius-1.2f, smallAnchorColor);
               int oppositeIndex = (i + 2) % 4;
               // big anchor on corners
               if (!gContext->mbUsingBounds && gContext->mbEnable && overBigAnchor && CanActivate())
               {
                   gContext->mBoundsPivot.TransformPoint(aabb[(i + 2) % 4], gContext->mModelSource);
                   gContext->mBoundsAnchor.TransformPoint(aabb[i], gContext->mModelSource);
                   gContext->mBoundsPlan = BuildPlan(gContext->mBoundsAnchor, bestAxisWorldDirection);
                   gContext->mBoundsBestAxis = bestAxis;
                   gContext->mBoundsAxis[0] = secondAxis;
                   gContext->mBoundsAxis[1] = thirdAxis;

                   gContext->mBoundsLocalPivot.Set(0.f);
                   gContext->mBoundsLocalPivot[secondAxis] = aabb[oppositeIndex][secondAxis];
                   gContext->mBoundsLocalPivot[thirdAxis] = aabb[oppositeIndex][thirdAxis];

                   gContext->mbUsingBounds = true;
                   gContext->mBoundsMatrix = gContext->mModelSource;
               }
               // small anchor on middle of segment
               if (!gContext->mbUsingBounds && gContext->mbEnable && overSmallAnchor && CanActivate())
               {
                   vec_t midPointOpposite = (aabb[(i + 2) % 4] + aabb[(i + 3) % 4]) * 0.5f;
                   gContext->mBoundsPivot.TransformPoint(midPointOpposite, gContext->mModelSource);
                   gContext->mBoundsAnchor.TransformPoint(midPoint, gContext->mModelSource);
                   gContext->mBoundsPlan = BuildPlan(gContext->mBoundsAnchor, bestAxisWorldDirection);
                   gContext->mBoundsBestAxis = bestAxis;
                   int indices[] = { secondAxis , thirdAxis };
                   gContext->mBoundsAxis[0] = indices[i%2];
                   gContext->mBoundsAxis[1] = -1;

                   gContext->mBoundsLocalPivot.Set(0.f);
                   gContext->mBoundsLocalPivot[gContext->mBoundsAxis[0]] = aabb[oppositeIndex][indices[i % 2]];// bounds[gContext->mBoundsAxis[0]] * (((i + 1) & 2) ? 1.f : -1.f);

                   gContext->mbUsingBounds = true;
                   gContext->mBoundsMatrix = gContext->mModelSource;
               }
           }

           if (gContext->mbUsingBounds)
           {
               matrix_t scale;
               scale.SetToIdentity();

               // compute projected mouse position on plan
               const float len = IntersectRayPlane(gContext->mRayOrigin, gContext->mRayVector, gContext->mBoundsPlan);
               vec_t newPos = gContext->mRayOrigin + gContext->mRayVector * len;

               // compute a reference and delta vectors base on mouse move
               vec_t deltaVector = (newPos - gContext->mBoundsPivot).Abs();
               vec_t referenceVector = (gContext->mBoundsAnchor - gContext->mBoundsPivot).Abs();

               // for 1 or 2 axes, compute a ratio that's used for scale and snap it based on resulting length
               for (int i = 0; i < 2; i++)
               {
                   int axisIndex1 = gContext->mBoundsAxis[i];
                   if (axisIndex1 == -1)
                       continue;

                   float ratioAxis = 1.f;
                   vec_t axisDir = gContext->mBoundsMatrix.component[axisIndex1].Abs();

                   float dtAxis = axisDir.Dot(referenceVector);
                   float boundSize = bounds[axisIndex1 + 3] - bounds[axisIndex1];
                   if (dtAxis > FLT_EPSILON)
                       ratioAxis = axisDir.Dot(deltaVector) / dtAxis;

                   if (snapValues)
                   {
                       float length = boundSize * ratioAxis;
                       ComputeSnap(&length, snapValues[axisIndex1]);
                       if (boundSize > FLT_EPSILON)
                           ratioAxis = length / boundSize;
                   }
                   scale.component[axisIndex1] *= ratioAxis;
               }

               // transform matrix
               matrix_t preScale, postScale;
               preScale.Translation(-gContext->mBoundsLocalPivot);
               postScale.Translation(gContext->mBoundsLocalPivot);
               matrix_t res = preScale * scale * postScale * gContext->mBoundsMatrix;
               *matrix = res;

               // info text
               char tmps[512];
               ImVec2 destinationPosOnScreen = worldToPos(gContext->mModel.v.position, gContext->mViewProjection);
               ImFormatString(tmps, sizeof(tmps), "X: %.2f Y: %.2f Z:%.2f"
                   , (bounds[3] - bounds[0]) * gContext->mBoundsMatrix.component[0].Length() * scale.component[0].Length()
                   , (bounds[4] - bounds[1]) * gContext->mBoundsMatrix.component[1].Length() * scale.component[1].Length()
                   , (bounds[5] - bounds[2]) * gContext->mBoundsMatrix.component[2].Length() * scale.component[2].Length()
               );
               drawList->AddText(ImVec2(destinationPosOnScreen.x + 15, destinationPosOnScreen.y + 15), 0xFF000000, tmps);
               drawList->AddText(ImVec2(destinationPosOnScreen.x + 14, destinationPosOnScreen.y + 14), 0xFFFFFFFF, tmps);
            }

           if (!io.MouseDown[0])
               gContext->mbUsingBounds = false;

           if( gContext->mbUsingBounds )
               break;
       }
   }

   ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   //

   static int GetScaleType()
   {
      ImGuiIO& io = ImGui::GetIO();
      int type = NONE;

      // screen
      if (io.MousePos.x >= gContext->mScreenSquareMin.x && io.MousePos.x <= gContext->mScreenSquareMax.x &&
         io.MousePos.y >= gContext->mScreenSquareMin.y && io.MousePos.y <= gContext->mScreenSquareMax.y)
         type = SCALE_XYZ;

      // compute
      for (unsigned int i = 0; i < 3 && type == NONE; i++)
      {
         vec_t dirPlaneX, dirPlaneY, dirAxis;
         bool belowAxisLimit, belowPlaneLimit;
         ComputeTripodAxisAndVisibility(i, dirAxis, dirPlaneX, dirPlaneY, belowAxisLimit, belowPlaneLimit);

       const float len = IntersectRayPlane(gContext->mRayOrigin, gContext->mRayVector, BuildPlan(gContext->mModel.v.position, dirAxis));
       vec_t posOnPlan = gContext->mRayOrigin + gContext->mRayVector * len;

       const ImVec2 posOnPlanScreen = worldToPos(posOnPlan, gContext->mViewProjection);
       const ImVec2 axisStartOnScreen = worldToPos(gContext->mModel.v.position + dirAxis * gContext->mScreenFactor * 0.1f, gContext->mViewProjection);
       const ImVec2 axisEndOnScreen = worldToPos(gContext->mModel.v.position + dirAxis * gContext->mScreenFactor, gContext->mViewProjection);

       vec_t closestPointOnAxis = PointOnSegment(makeVect(posOnPlanScreen), makeVect(axisStartOnScreen), makeVect(axisEndOnScreen));

       if ((closestPointOnAxis - makeVect(posOnPlanScreen)).Length() < 12.f) // pixel size
          type = SCALE_X + i;
      }
      return type;
   }

   static int GetRotateType()
   {
      ImGuiIO& io = ImGui::GetIO();
      int type = NONE;

      vec_t deltaScreen = { io.MousePos.x - gContext->mScreenSquareCenter.x, io.MousePos.y - gContext->mScreenSquareCenter.y, 0.f, 0.f };
      float dist = deltaScreen.Length();
      if (dist >= (gContext->mRadiusSquareCenter - 1.0f) && dist < (gContext->mRadiusSquareCenter + 1.0f))
         type = ROTATE_SCREEN;

      const vec_t planNormals[] = { gContext->mModel.v.right, gContext->mModel.v.up, gContext->mModel.v.dir};

      for (unsigned int i = 0; i < 3 && type == NONE; i++)
      {
         // pickup plan
         vec_t pickupPlan = BuildPlan(gContext->mModel.v.position, planNormals[i]);

         const float len = IntersectRayPlane(gContext->mRayOrigin, gContext->mRayVector, pickupPlan);
         vec_t localPos = gContext->mRayOrigin + gContext->mRayVector * len - gContext->mModel.v.position;

         if (Dot(Normalized(localPos), gContext->mRayVector) > FLT_EPSILON)
            continue;
       vec_t idealPosOnCircle = Normalized(localPos);
       idealPosOnCircle.TransformVector(gContext->mModelInverse);
       ImVec2 idealPosOnCircleScreen = worldToPos(idealPosOnCircle * gContext->mScreenFactor, gContext->mMVP);

       //gContext->mDrawList->AddCircle(idealPosOnCircleScreen, 5.f, 0xFFFFFFFF);
       ImVec2 distanceOnScreen = idealPosOnCircleScreen - io.MousePos;

         float distance = makeVect(distanceOnScreen).Length();
         if (distance < 8.f) // pixel size
            type = ROTATE_X + i;
      }

      return type;
   }

   static int GetMoveType(vec_t *gizmoHitProportion)
   {
      ImGuiIO& io = ImGui::GetIO();
      int type = NONE;

      // screen
      if (io.MousePos.x >= gContext->mScreenSquareMin.x && io.MousePos.x <= gContext->mScreenSquareMax.x &&
         io.MousePos.y >= gContext->mScreenSquareMin.y && io.MousePos.y <= gContext->mScreenSquareMax.y)
         type = MOVE_SCREEN;

      // compute
      for (unsigned int i = 0; i < 3 && type == NONE; i++)
      {
         vec_t dirPlaneX, dirPlaneY, dirAxis;
         bool belowAxisLimit, belowPlaneLimit;
         ComputeTripodAxisAndVisibility(i, dirAxis, dirPlaneX, dirPlaneY, belowAxisLimit, belowPlaneLimit);
       dirAxis.TransformVector(gContext->mModel);
         dirPlaneX.TransformVector(gContext->mModel);
         dirPlaneY.TransformVector(gContext->mModel);

         const float len = IntersectRayPlane(gContext->mRayOrigin, gContext->mRayVector, BuildPlan(gContext->mModel.v.position, dirAxis));
         vec_t posOnPlan = gContext->mRayOrigin + gContext->mRayVector * len;

       const ImVec2 posOnPlanScreen = worldToPos(posOnPlan, gContext->mViewProjection);
       const ImVec2 axisStartOnScreen = worldToPos(gContext->mModel.v.position + dirAxis * gContext->mScreenFactor * 0.1f, gContext->mViewProjection);
       const ImVec2 axisEndOnScreen = worldToPos(gContext->mModel.v.position + dirAxis * gContext->mScreenFactor, gContext->mViewProjection);

       vec_t closestPointOnAxis = PointOnSegment(makeVect(posOnPlanScreen), makeVect(axisStartOnScreen), makeVect(axisEndOnScreen));

       if ((closestPointOnAxis - makeVect(posOnPlanScreen)).Length() < 12.f) // pixel size
            type = MOVE_X + i;

       const float dx = dirPlaneX.Dot3((posOnPlan - gContext->mModel.v.position) * (1.f / gContext->mScreenFactor));
       const float dy = dirPlaneY.Dot3((posOnPlan - gContext->mModel.v.position) * (1.f / gContext->mScreenFactor));
         if (belowPlaneLimit && dx >= quadUV[0] && dx <= quadUV[4] && dy >= quadUV[1] && dy <= quadUV[3])
            type = MOVE_YZ + i;

         if (gizmoHitProportion)
            *gizmoHitProportion = makeVect(dx, dy, 0.f);
      }
      return type;
   }

   static void HandleTranslation(float *matrix, float *deltaMatrix, int& type, float *snap)
   {
      ImGuiIO& io = ImGui::GetIO();
      bool applyRotationLocaly = gContext->mMode == LOCAL || type == MOVE_SCREEN;

      // move
      if (gContext->mbUsing)
      {
         ImGui::CaptureMouseFromApp();
         const float len = fabsf(IntersectRayPlane(gContext->mRayOrigin, gContext->mRayVector, gContext->mTranslationPlan)); // near plan
         vec_t newPos = gContext->mRayOrigin + gContext->mRayVector * len;



         // compute delta
         vec_t newOrigin = newPos - gContext->mRelativeOrigin * gContext->mScreenFactor;
         vec_t delta = newOrigin - gContext->mModel.v.position;

         // 1 axis constraint
         if (gContext->mCurrentOperation >= MOVE_X && gContext->mCurrentOperation <= MOVE_Z)
         {
            int axisIndex = gContext->mCurrentOperation - MOVE_X;
            const vec_t& axisValue = *(vec_t*)&gContext->mModel.m[axisIndex];
            float lengthOnAxis = Dot(axisValue, delta);
            delta = axisValue * lengthOnAxis;
         }

         // snap
         if (snap)
         {
            vec_t cumulativeDelta = gContext->mModel.v.position + delta - gContext->mMatrixOrigin;
            if (applyRotationLocaly)
            {
               matrix_t modelSourceNormalized = gContext->mModelSource;
               modelSourceNormalized.OrthoNormalize();
               matrix_t modelSourceNormalizedInverse;
               modelSourceNormalizedInverse.Inverse(modelSourceNormalized);
               cumulativeDelta.TransformVector(modelSourceNormalizedInverse);
               ComputeSnap(cumulativeDelta, snap);
               cumulativeDelta.TransformVector(modelSourceNormalized);
            }
            else
            {
               ComputeSnap(cumulativeDelta, snap);
            }
            delta = gContext->mMatrixOrigin + cumulativeDelta - gContext->mModel.v.position;

         }

         // compute matrix & delta
         matrix_t deltaMatrixTranslation;
         deltaMatrixTranslation.Translation(delta);
         if (deltaMatrix)
            memcpy(deltaMatrix, deltaMatrixTranslation.m16, sizeof(float) * 16);


         matrix_t res = gContext->mModelSource * deltaMatrixTranslation;
         *(matrix_t*)matrix = res;

         if (!io.MouseDown[0])
            gContext->mbUsing = false;

         type = gContext->mCurrentOperation;
      }
      else
      {
         // find new possible way to move
         vec_t gizmoHitProportion;
         type = GetMoveType(&gizmoHitProportion);
         if(type != NONE)
         {
            ImGui::CaptureMouseFromApp();
         }
       if (CanActivate() && type != NONE)
       {
          gContext->mbUsing = true;
          gContext->mCurrentOperation = type;
          vec_t movePlanNormal[] = { gContext->mModel.v.right, gContext->mModel.v.up, gContext->mModel.v.dir,
             gContext->mModel.v.right, gContext->mModel.v.up, gContext->mModel.v.dir,
             -gContext->mCameraDir };

          vec_t cameraToModelNormalized = Normalized(gContext->mModel.v.position - gContext->mCameraEye);
          for (unsigned int i = 0; i < 3; i++)
          {
             vec_t orthoVector = Cross(movePlanNormal[i], cameraToModelNormalized);
             movePlanNormal[i].Cross(orthoVector);
             movePlanNormal[i].Normalize();
          }
            // pickup plan
            gContext->mTranslationPlan = BuildPlan(gContext->mModel.v.position, movePlanNormal[type - MOVE_X]);
            const float len = IntersectRayPlane(gContext->mRayOrigin, gContext->mRayVector, gContext->mTranslationPlan);
            gContext->mTranslationPlanOrigin = gContext->mRayOrigin + gContext->mRayVector * len;
            gContext->mMatrixOrigin = gContext->mModel.v.position;

            gContext->mRelativeOrigin = (gContext->mTranslationPlanOrigin - gContext->mModel.v.position) * (1.f / gContext->mScreenFactor);
         }
      }
   }

   static void HandleScale(float *matrix, float *deltaMatrix, int& type, float *snap)
   {
      ImGuiIO& io = ImGui::GetIO();

      if (!gContext->mbUsing)
      {
         // find new possible way to scale
         type = GetScaleType();
         if(type != NONE)
         {
            ImGui::CaptureMouseFromApp();
         }
         if (CanActivate() && type != NONE)
         {
            gContext->mbUsing = true;
            gContext->mCurrentOperation = type;
            const vec_t movePlanNormal[] = { gContext->mModel.v.up, gContext->mModel.v.dir, gContext->mModel.v.right, gContext->mModel.v.dir, gContext->mModel.v.up, gContext->mModel.v.right, -gContext->mCameraDir };
            // pickup plan

            gContext->mTranslationPlan = BuildPlan(gContext->mModel.v.position, movePlanNormal[type - SCALE_X]);
            const float len = IntersectRayPlane(gContext->mRayOrigin, gContext->mRayVector, gContext->mTranslationPlan);
            gContext->mTranslationPlanOrigin = gContext->mRayOrigin + gContext->mRayVector * len;
            gContext->mMatrixOrigin = gContext->mModel.v.position;
            gContext->mScale.Set(1.f, 1.f, 1.f);
            gContext->mRelativeOrigin = (gContext->mTranslationPlanOrigin - gContext->mModel.v.position) * (1.f / gContext->mScreenFactor);
            gContext->mScaleValueOrigin = makeVect(gContext->mModelSource.v.right.Length(), gContext->mModelSource.v.up.Length(), gContext->mModelSource.v.dir.Length());
            gContext->mSaveMousePosx = io.MousePos.x;
         }
      }
      // scale
      if (gContext->mbUsing)
      {
         ImGui::CaptureMouseFromApp();
         const float len = IntersectRayPlane(gContext->mRayOrigin, gContext->mRayVector, gContext->mTranslationPlan);
         vec_t newPos = gContext->mRayOrigin + gContext->mRayVector * len;
         vec_t newOrigin = newPos - gContext->mRelativeOrigin * gContext->mScreenFactor;
         vec_t delta = newOrigin - gContext->mModel.v.position;

         // 1 axis constraint
         if (gContext->mCurrentOperation >= SCALE_X && gContext->mCurrentOperation <= SCALE_Z)
         {
            int axisIndex = gContext->mCurrentOperation - SCALE_X;
            const vec_t& axisValue = *(vec_t*)&gContext->mModel.m[axisIndex];
            float lengthOnAxis = Dot(axisValue, delta);
            delta = axisValue * lengthOnAxis;

            vec_t baseVector = gContext->mTranslationPlanOrigin - gContext->mModel.v.position;
            float ratio = Dot(axisValue, baseVector + delta) / Dot(axisValue, baseVector);

            gContext->mScale[axisIndex] = max(ratio, 0.001f);
         }
         else
         {
            float scaleDelta = (io.MousePos.x - gContext->mSaveMousePosx)  * 0.01f;
            gContext->mScale.Set(max(1.f + scaleDelta, 0.001f));
         }

         // snap
         if (snap)
         {
            float scaleSnap[] = { snap[0], snap[0], snap[0] };
            ComputeSnap(gContext->mScale, scaleSnap);
         }

         // no 0 allowed
         for (int i = 0; i < 3;i++)
            gContext->mScale[i] = max(gContext->mScale[i], 0.001f);

         // compute matrix & delta
         matrix_t deltaMatrixScale;
         deltaMatrixScale.Scale(gContext->mScale * gContext->mScaleValueOrigin);

         matrix_t res = deltaMatrixScale * gContext->mModel;
         *(matrix_t*)matrix = res;

         if (deltaMatrix)
         {
            deltaMatrixScale.Scale(gContext->mScale);
            memcpy(deltaMatrix, deltaMatrixScale.m16, sizeof(float) * 16);
         }

         if (!io.MouseDown[0])
            gContext->mbUsing = false;

         type = gContext->mCurrentOperation;
      }
   }

   static void HandleRotation(float *matrix, float *deltaMatrix, int& type, float *snap)
   {
      ImGuiIO& io = ImGui::GetIO();
      bool applyRotationLocaly = gContext->mMode == LOCAL;

      if (!gContext->mbUsing)
      {
         type = GetRotateType();

         if(type != NONE)
         {
            ImGui::CaptureMouseFromApp();
         }

         if (type == ROTATE_SCREEN)
         {
            applyRotationLocaly = true;
         }

         if (CanActivate() && type != NONE)
         {
            gContext->mbUsing = true;
            gContext->mCurrentOperation = type;
            const vec_t rotatePlanNormal[] = { gContext->mModel.v.right, gContext->mModel.v.up, gContext->mModel.v.dir, -gContext->mCameraDir };
            // pickup plan
            if (applyRotationLocaly)
            {
               gContext->mTranslationPlan = BuildPlan(gContext->mModel.v.position, rotatePlanNormal[type - ROTATE_X]);
            }
            else
            {
               gContext->mTranslationPlan = BuildPlan(gContext->mModelSource.v.position, directionUnary[type - ROTATE_X]);
            }

            const float len = IntersectRayPlane(gContext->mRayOrigin, gContext->mRayVector, gContext->mTranslationPlan);
            vec_t localPos = gContext->mRayOrigin + gContext->mRayVector * len - gContext->mModel.v.position;
            gContext->mRotationVectorSource = Normalized(localPos);
            gContext->mRotationAngleOrigin = ComputeAngleOnPlan();
         }
      }

      // rotation
      if (gContext->mbUsing)
      {
         ImGui::CaptureMouseFromApp();
         gContext->mRotationAngle = ComputeAngleOnPlan();
         if (snap)
         {
            float snapInRadian = snap[0] * DEG2RAD;
            ComputeSnap(&gContext->mRotationAngle, snapInRadian);
         }
         vec_t rotationAxisLocalSpace;

         rotationAxisLocalSpace.TransformVector(makeVect(gContext->mTranslationPlan.x, gContext->mTranslationPlan.y, gContext->mTranslationPlan.z, 0.f), gContext->mModelInverse);
         rotationAxisLocalSpace.Normalize();

         matrix_t deltaRotation;
         deltaRotation.RotationAxis(rotationAxisLocalSpace, gContext->mRotationAngle - gContext->mRotationAngleOrigin);
         gContext->mRotationAngleOrigin = gContext->mRotationAngle;

         matrix_t scaleOrigin;
         scaleOrigin.Scale(gContext->mModelScaleOrigin);

         if (applyRotationLocaly)
         {
            *(matrix_t*)matrix = scaleOrigin * deltaRotation * gContext->mModel;
         }
         else
         {
            matrix_t res = gContext->mModelSource;
            res.v.position.Set(0.f);

            *(matrix_t*)matrix = res * deltaRotation;
            ((matrix_t*)matrix)->v.position = gContext->mModelSource.v.position;
         }

         if (deltaMatrix)
         {
            *(matrix_t*)deltaMatrix = gContext->mModelInverse * deltaRotation * gContext->mModel;
         }

         if (!io.MouseDown[0])
            gContext->mbUsing = false;

         type = gContext->mCurrentOperation;
      }
   }

   void DecomposeMatrixToComponents(const float *matrix, float *translation, float *rotation, float *scale)
   {
      matrix_t mat = *(matrix_t*)matrix;

      scale[0] = mat.v.right.Length();
      scale[1] = mat.v.up.Length();
      scale[2] = mat.v.dir.Length();

      mat.OrthoNormalize();

      rotation[0] = RAD2DEG * atan2f(mat.m[1][2], mat.m[2][2]);
      rotation[1] = RAD2DEG * atan2f(-mat.m[0][2], sqrtf(mat.m[1][2] * mat.m[1][2] + mat.m[2][2]* mat.m[2][2]));
      rotation[2] = RAD2DEG * atan2f(mat.m[0][1], mat.m[0][0]);

      translation[0] = mat.v.position.x;
      translation[1] = mat.v.position.y;
      translation[2] = mat.v.position.z;
   }

   void RecomposeMatrixFromComponents(const float *translation, const float *rotation, const float *scale, float *matrix)
   {
      matrix_t& mat = *(matrix_t*)matrix;

      matrix_t rot[3];
      for (int i = 0; i < 3;i++)
         rot[i].RotationAxis(directionUnary[i], rotation[i] * DEG2RAD);

      mat = rot[0] * rot[1] * rot[2];

      float validScale[3];
      for (int i = 0; i < 3; i++)
      {
         if (fabsf(scale[i]) < FLT_EPSILON)
            validScale[i] = 0.001f;
         else
            validScale[i] = scale[i];
      }
      mat.v.right *= validScale[0];
      mat.v.up *= validScale[1];
      mat.v.dir *= validScale[2];
      mat.v.position.Set(translation[0], translation[1], translation[2], 1.f);
   }

   void Manipulate(const float *view, const float *projection, OPERATION operation, MODE mode, float *matrix, float *deltaMatrix, float *snap, float *localBounds, float *boundsSnap)
   {
      ComputeContext(view, projection, matrix, mode);

      // set delta to identity
      if (deltaMatrix)
         ((matrix_t*)deltaMatrix)->SetToIdentity();

      // behind camera
      vec_t camSpacePosition;
      camSpacePosition.TransformPoint(makeVect(0.f, 0.f, 0.f), gContext->mMVP);
      if (!gContext->mIsOrthographic && camSpacePosition.z < 0.001f)
         return;

      // --
      int type = NONE;
      if (gContext->mbEnable)
      {
          if (!gContext->mbUsingBounds)
          {
              switch (operation)
              {
              case ROTATE:
                  HandleRotation(matrix, deltaMatrix, type, snap);
                  break;
              case TRANSLATE:
                  HandleTranslation(matrix, deltaMatrix, type, snap);
                  break;
              case SCALE:
                  HandleScale(matrix, deltaMatrix, type, snap);
                  break;
              case BOUNDS:
                  break;
              }
          }
      }

      if (localBounds && !gContext->mbUsing)
          HandleAndDrawLocalBounds(localBounds, (matrix_t*)matrix, boundsSnap, operation);

      if (!gContext->mbUsingBounds)
      {
          switch (operation)
          {
          case ROTATE:
              DrawRotationGizmo(type);
              break;
          case TRANSLATE:
              DrawTranslationGizmo(type);
              break;
          case SCALE:
              DrawScaleGizmo(type);
              break;
          case BOUNDS:
              break;
          }
      }
   }

   void DrawCube(const float *view, const float *projection, const float *matrix)
   {
      matrix_t viewInverse;
      viewInverse.Inverse(*(matrix_t*)view);
      const matrix_t& model = *(matrix_t*)matrix;
      matrix_t res = *(matrix_t*)matrix * *(matrix_t*)view * *(matrix_t*)projection;

      for (int iFace = 0; iFace < 6; iFace++)
      {
         const int normalIndex = (iFace % 3);
         const int perpXIndex = (normalIndex + 1) % 3;
         const int perpYIndex = (normalIndex + 2) % 3;
         const float invert = (iFace > 2) ? -1.f : 1.f;

         const vec_t faceCoords[4] = { directionUnary[normalIndex] + directionUnary[perpXIndex] + directionUnary[perpYIndex],
            directionUnary[normalIndex] + directionUnary[perpXIndex] - directionUnary[perpYIndex],
            directionUnary[normalIndex] - directionUnary[perpXIndex] - directionUnary[perpYIndex],
            directionUnary[normalIndex] - directionUnary[perpXIndex] + directionUnary[perpYIndex],
         };

         // clipping
         bool skipFace = false;
         for (unsigned int iCoord = 0; iCoord < 4; iCoord++)
         {
            vec_t camSpacePosition;
            camSpacePosition.TransformPoint(faceCoords[iCoord] * 0.5f * invert, gContext->mMVP);
            if (camSpacePosition.z < 0.001f)
            {
               skipFace = true;
               break;
            }
         }
         if (skipFace)
            continue;

         // 3D->2D
         ImVec2 faceCoordsScreen[4];
         for (unsigned int iCoord = 0; iCoord < 4; iCoord++)
            faceCoordsScreen[iCoord] = worldToPos(faceCoords[iCoord] * 0.5f * invert, res);

         // back face culling
         vec_t cullPos, cullNormal;
         cullPos.TransformPoint(faceCoords[0] * 0.5f * invert, model);
         cullNormal.TransformVector(directionUnary[normalIndex] * invert, model);
         float dt = Dot(Normalized(cullPos - viewInverse.v.position), Normalized(cullNormal));
         if (dt>0.f)
            continue;

         // draw face with lighter color
         gContext->mDrawList->AddConvexPolyFilled(faceCoordsScreen, 4, directionColor[normalIndex] | 0x808080);
      }
   }

   void DrawGrid(const float *view, const float *projection, const float *matrix, const float gridSize)
   {
      matrix_t res = *(matrix_t*)matrix * *(matrix_t*)view * *(matrix_t*)projection;

      for (float f = -gridSize; f <= gridSize; f += 1.f)
      {
         gContext->mDrawList->AddLine(worldToPos(makeVect(f, 0.f, -gridSize), res), worldToPos(makeVect(f, 0.f, gridSize), res), 0xFF808080);
         gContext->mDrawList->AddLine(worldToPos(makeVect(-gridSize, 0.f, f), res), worldToPos(makeVect(gridSize, 0.f, f), res), 0xFF808080);
      }
   }
};

