// ----------------------------------------------------------------------------
// -- Element Toolkit (id.elemtk) — generic editor wrapper primitives        --
// ----------------------------------------------------------------------------
#if UNITY_EDITOR
using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace id.elemtk.Framework
{
    /// <summary>
    /// Generic sub-asset / composite enumeration over Unity asset containers.
    ///
    /// Most Unity asset-graph assets (MxM AnimData + AnimationModules +
    /// composites, VFX graphs, serialized dictionaries) store their parts as
    /// sub-assets of one container file. This utility turns
    /// AssetDatabase.LoadAllAssetsAtPath into a typed, filterable census.
    /// </summary>
    public static class SubAssetUtility
    {
        /// <summary>All sub-assets of a container file matching a type
        /// (assignable). Returns a fresh list; safe with empty/missing paths.</summary>
        public static List<T> Enumerate<T>(string containerPath) where T : UnityEngine.Object
        {
            var result = new List<T>();
            if (string.IsNullOrEmpty(containerPath))
                return result;

            var all = AssetDatabase.LoadAllAssetsAtPath(containerPath);
            if (all == null)
                return result;

            for (int i = 0; i < all.Length; i++)
            {
                if (all[i] is T typed)
                    result.Add(typed);
            }

            return result;
        }

        /// <summary>All container files (by extension filter) under a folder,
        /// recursively, that contain at least one sub-asset of type T.</summary>
        public static List<string> FindContainers<T>(string folder, string searchPattern = "*.asset")
            where T : UnityEngine.Object
        {
            var result = new List<string>();
            if (string.IsNullOrEmpty(folder))
                return result;

            var guids = AssetDatabase.FindAssets($"t:Object", new[] { folder });
            var seen = new HashSet<string>();

            for (int i = 0; i < guids.Length; i++)
            {
                string path = AssetDatabase.GUIDToAssetPath(guids[i]);
                if (string.IsNullOrEmpty(path) || !seen.Add(path))
                    continue;

                if (System.IO.Path.GetExtension(path) != System.IO.Path.GetExtension(searchPattern))
                    continue;

                // Cheap containment test: one load, type check, keep main assets out.
                var all = AssetDatabase.LoadAllAssetsAtPath(path);
                if (all == null)
                    continue;

                for (int k = 0; k < all.Length; k++)
                {
                    // Sub-assets have a main object that differs from themselves.
                    if (all[k] is T && AssetDatabase.IsMainAsset(all[k]) == false)
                    {
                        result.Add(path);
                        break;
                    }
                }
            }

            return result;
        }

        /// <summary>Combined version hash of a set of assets (content or
        /// dependency hash) — cheap enough to poll each frame-ish.</summary>
        public static string VersionHash(IEnumerable<string> paths)
        {
            var hash = new Hash128();
            int count = 0;

            foreach (string path in paths)
            {
                if (string.IsNullOrEmpty(path) || !AssetDatabase.LoadMainAssetAtPath(path))
                    continue;

                var assetHash = AssetDatabase.GetAssetDependencyHash(path);
                hash.Append(ref assetHash);
                count++;
            }

            return count > 0 ? hash.ToString() : string.Empty;
        }
    }
}
#endif