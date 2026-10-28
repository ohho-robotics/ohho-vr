using System.IO;
using System.Linq;
using NUnit.Framework;
using OmniBot.VR.AprilTagSpike;
using OmniBot.VR.Detection;
using UnityEngine;

namespace OmniBot.VR.Tests
{
    public class PermissionRationaleTests
    {
        [Test]
        public void Notices_ExplainHeadsetCameraAndHandTracking()
        {
            Assert.AreEqual(2, HeadsetCameraGate.Notices.Count);

            var camera = HeadsetCameraGate.Notices.Single(notice => notice.RequestedAtRuntime);
            Assert.AreEqual(HeadsetCameraPermission.PermissionId, camera.PermissionId);
            Assert.AreEqual("horizonos.permission.HEADSET_CAMERA", camera.PermissionId);
            Assert.AreEqual(HeadsetCameraGate.Rationale, camera.Rationale);
            StringAssert.Contains("camera", camera.Rationale.ToLowerInvariant());
            StringAssert.Contains("robot", camera.Rationale.ToLowerInvariant());
            StringAssert.Contains("headset", camera.Rationale.ToLowerInvariant());

            var hands = HeadsetCameraGate.Notices.Single(notice => !notice.RequestedAtRuntime);
            Assert.AreEqual(HeadsetCameraGate.HandTrackingPermissionId, hands.PermissionId);
            Assert.AreEqual(HeadsetCameraGate.HandTrackingRationale, hands.Rationale);
            StringAssert.Contains("hand", hands.Rationale.ToLowerInvariant());

            StringAssert.Contains("manual", HeadsetCameraGate.DeniedReason.ToLowerInvariant());
            StringAssert.Contains("denied", HeadsetCameraGate.DeniedReason.ToLowerInvariant());
            StringAssert.Contains("not used", HeadsetCameraGate.DeniedReason.ToLowerInvariant());
        }

        [Test]
        public void AndroidStringResources_MatchTheRuntimeCopy()
        {
            var plugins = Path.Combine(Application.dataPath, "Plugins", "Android");
            // Unity 6 fails the player build when this directory exists.
            Assert.IsFalse(Directory.Exists(Path.Combine(plugins, "res")));

            var library = Path.Combine(plugins, "PermissionRationales.androidlib");
            var path = Path.Combine(library, "res", "values", "strings.xml");
            Assert.IsTrue(File.Exists(path), path);
            var xml = File.ReadAllText(path);
            StringAssert.Contains(HeadsetCameraGate.Rationale, xml);
            StringAssert.Contains(HeadsetCameraGate.HandTrackingRationale, xml);
            StringAssert.Contains(HeadsetCameraGate.DeniedReason, xml);

            var manifestPath = Path.Combine(library, "AndroidManifest.xml");
            Assert.IsTrue(File.Exists(manifestPath), manifestPath);
            StringAssert.Contains(
                "package=\"com.ohhorobotics.ohhovr.permissions\"",
                File.ReadAllText(manifestPath));
        }
    }
}
