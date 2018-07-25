/*
 This is the educational project with main purpose is to develop and test the multidimensional 
 adaptive grid which can be used in Reinforcement Learning area. 
 In typical reinforcement learning scenario an agent takes actions corresponding to current 
 policy and state of an environment. Adaptive grid is needed to represent a manifold of possible
 actions of an agent and manifold of states of an environment with ability to adjust grid resolution
 in regions of interest. The concept of adaptive refinement of the manifolds of possible states and
 actions is close to common learning practices (for example, in learning of car driving
 at a very first stages there is no need for any distinction of severity of bends, instead, 
 the only distinction between left and right turns could be suitable. The more experience and 
 average speeds, the more details about bends and the more precise steering are needed for further
 improvement of driving skills).
 
 The class AdaptiveGrid is implemented as voxel tree. Each voxel exposes its properties 
 and methods through the IVoxel interface. The maximum number of voxels and shape of the grid
 are set at creation time of the AdaptiveGrid instance and can't be changed during the liftime
 of the instance. The voxels are stored in the internal single-dimensional array, which entirely
 initialized at creation time of the AdaptiveGrid instance. So, during different manipulations
 with the grid (voxel tessellation or merging), voxels only are changed their properties and
 no voxel instances are created or deleted. The grid can be processed in multithreaded manner.

 2018 zubetto85@gmail.com
 */

using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;

namespace Cognita
{
    public class AdaptiveGrid<T>
    {
        public interface IVoxel
        {
            /// <summary>
            /// True indicates that this voxel is in use
            /// </summary>
            bool IsActive { get; }

            /// <summary>
            /// Is used together with AdaptiveGrid instance property ProcessedState
            /// to mark whether this voxel was processed or not
            /// </summary>
            bool State { get; set; }

            /// <summary>
            /// Index of this voxel in the InternalArray 
            /// </summary>
            int SerialIndex { get; }

            /// <summary>
            /// Level of tesselation; root-voxels are located at the zero level
            /// </summary>
            int TessLevel { get; }

            /// <summary>
            /// Serial index of the first child voxel; -1 if this voxel has no children
            /// </summary>
            int ContentSI { get; }

            /// <summary>
            /// Ancestor of this voxel; null if this voxel is root-voxel 
            /// or if this voxel is not used (IsActive equals to false)
            /// </summary>
            IVoxel Ancestor { get; }

            /// <summary>
            /// Coordinates of the origin of this voxel
            /// </summary>
            IList<double> Origin { get; }

            /// <summary>
            /// The size of this voxel
            /// </summary>
            IList<double> GridStep { get; }

            /// <summary>
            /// Calculates coordinates of the voxel center
            /// </summary>
            /// <param name="point">The array into which coords will be placed</param>
            void GetCenter(double[] point);

            /// <summary>
            /// Calculates coordinates of the center of the first direct child
            /// </summary>
            /// <param name="point">The array into which vector increment will be placed</param>
            void GetFirstCenter(double[] point);

            /// <summary>
            /// Copies the coords of the origin of this voxel to the given "point" array
            /// starting at the given offset; O(1)
            /// </summary>
            /// <param name="point">The array into which coords will be copied</param>
            /// <param name="offset">The zero-based byte offset into "point"</param>
            void CopyOrigin(double[] point, int offset = 0);

            /// <summary>
            /// Copies the size of this voxel to the given "step" array
            /// starting at the given offset; O(1)
            /// </summary>
            /// <param name="step">The array into which coords will be copied</param>
            /// <param name="offset">The zero-based byte offset into "step"</param>
            void CopyGridStep(double[] step, int offset = 0);

            /// <summary>
            /// Copies the origin coords and size of this voxel to the given "placement" array
            /// starting at the given offset; O(1)
            /// </summary>
            /// <param name="placement">The array into which coords and size will be copied</param>
            /// <param name="offset">The zero-based byte offset into "offset"</param>
            void GetPlacement(double[] placement, int offset = 0);

            /// <summary>
            /// This value should considered as actual if at least this voxel is active
            /// </summary>
            T Data { get; set; }

            /// <summary>
            /// Returns true only if this voxel has been successfully tessellated;
            /// Any voxel can be tessellated only if it is active and it has not been already
            /// tessellated and there are sufficient number of inactive voxels in the internal array;
            /// Thread-safe
            /// </summary>
            /// <param name="mandatorily">If true then thread will be blocked until other threads will end up processing of this voxel</param>
            /// <param name="initializer">The data of each child will be set using this method</param>
            /// <param name="iniValue">The data of all children will get this value, only if initializer is null</param>
            /// <returns></returns>
            bool Tessellate(bool mandatorily, Func <IVoxel, T> initializer = null, T iniValue = default(T));

            /// <summary>
            /// Returns true only if this voxel has been successfully tessellated;
            /// Any voxel can be tessellated only if it is active and it has not been already
            /// tessellated and there are sufficient number of inactive voxels in the internal array;
            /// First, tries to reserve slots for the tessellation, then if success, evaluates the scan;
            /// Thread-safe
            /// </summary>
            /// <param name="mandatorily">If true then thread will be blocked until other threads will end up processing of this voxel</param>
            /// <param name="scan">Tessellation is performed if the scan is evaluated to true and all other conditions are met</param>
            /// <param name="initializer">The data of each child will be set using this method</param>
            /// <param name="iniValue">The data of all children will get this value, only if initializer is null</param>
            /// <returns></returns>
            bool Tessellate(bool mandatorily, Predicate<IVoxel> scan, Func<IVoxel, T> initializer, T iniValue = default(T));

            /// <summary>
            /// Returns true only if this voxel has been successfully tessellated;
            /// Any voxel can be tessellated only if it is active and it has not been already
            /// tessellated and there are sufficient number of inactive voxels in the internal array;
            /// First, tries to reserve slots for the tessellation, then if success, evaluates the scan;
            /// This method is useful if the scan is responsible for the children data;
            /// Thread-safe
            /// </summary>
            /// <param name="mandatorily">If true then thread will be blocked until other threads will end up processing of this voxel</param>
            /// <param name="scan">Tessellation is performed if the predicate is true and all other conditions are met</param>
            /// <returns></returns>
            bool Tessellate(bool mandatorily, Predicate<IVoxel> scan);

            /// <summary>
            /// Eliminates information about the content of this voxel
            /// by non-recursive traversal of the whole voxel subtree;
            /// All voxels of the subtree will be set as inactive; Thread-safe 
            /// </summary>
            /// <param name="mandatorily">If true then thread will be blocked until other threads will end up processing of this voxel</param>
            /// <param name="initializer">The data of this voxel will be set using this method</param>
            /// <param name="iniValue">The data of this voxel will get this value, only if initializer is null</param>
            /// <returns></returns>
            bool Merge(bool mandatorily, Func<IVoxel, T> initializer = null, T iniValue = default(T));

            /// <summary>
            /// Eliminates information about the content of this voxel
            /// by non-recursive traversal of the whole voxel subtree;
            /// All voxels of the subtree will be set as inactive; Thread-safe 
            /// </summary>
            /// <param name="mandatorily">If true then thread will be blocked until other threads will end up processing of this voxel</param>
            /// <returns></returns>
            bool Merge(bool mandatorily);

            /// <summary>
            /// Eliminates information about the content of this voxel
            /// by recursive traversal of the whole voxel subtree;
            /// All voxels of the subtree will be set as inactive; Thread-safe 
            /// </summary>
            /// <param name="mandatorily">If true then thread will be blocked until other threads will end up processing of this voxel</param>
            /// <param name="initializer">The data of this voxel will be set using this method</param>
            /// <param name="iniValue">The data of this voxel will get this value, only if initializer is null</param>
            /// <returns></returns>
            bool Merge_R(bool mandatorily, Func<IVoxel, T> initializer = null, T iniValue = default(T));

            /// <summary>
            /// Sets the data for the first generation of children of this voxel and returns true
            /// if success; Thread-safe
            /// </summary>
            /// <param name="mandatorily">If true then thread will be blocked until other threads will end up processing of this voxel</param>
            /// <param name="initializer">The data of each child will be set using this method</param>
            /// <param name="iniValue">The data of all children will get this value, only if initializer is null</param>
            /// <returns></returns>
            bool SetContent(bool mandatorily, Func<IVoxel, T> initializer = null, T iniValue = default(T));

            /// <summary>
            /// Performs traversal of the point over the origins of direct childs of this voxel;
            /// after passing the last child voxel the point is positioned 
            /// at the origin of this voxel, index is reset to zero and function returns false
            /// </summary>
            /// <param name="index">Current index-vector; will be incremented</param>
            /// <param name="point">Current child voxel origin; will be shifted by the grid step of this voxel</param>
            /// <returns></returns>
            bool NextOrigin(int[] index, double[] point);

            /// <summary>
            /// Performs traversal of the point over the centers of direct childs of the given voxel;
            /// after passing the last child voxel the point is positioned at the center
            /// of the first child of this voxel, index is reset to zero and function returns false
            /// </summary>
            /// <param name="index">Current index-vector; will be incremented</param>
            /// <param name="point">Current child voxel center; will be shifted by the grid step of this voxel</param>
            /// <returns></returns>
            bool NextCenter(int[] index, double[] point);
        }

        private class Voxel : IVoxel
        {
            public bool isFree;
            public bool state = true;

            public int SI; // this voxel index
            public int Level; // hierarchical level is used to define gridSteps
            public int contentSI = -1; // the first index of the content; negative if no content

            public double[] gridOrigin;

            public AdaptiveGrid<T> root;
            public Voxel ancestor;

            public bool IsActive { get { return !isFree; } }
            public int SerialIndex { get { return SI; } }
            public int TessLevel { get { return Level; } }
            public int ContentSI { get { return contentSI; } }
            public IVoxel Ancestor { get { return ancestor; } }

            public bool State
            {
                get { lock (locker) return state; }
                set { lock (locker) state = value; }
            }

            public T Data { get; set; }
            
            public IList<double> Origin { get { return Array.AsReadOnly(gridOrigin); } }
            public IList<double> GridStep { get { return Array.AsReadOnly(root.scaledSteps[Level]); } }

            public void GetCenter(double[] point)
            {
                Buffer.BlockCopy(root.scaledSteps[Level], 0, point, 0, root.VectorSize);

                point.MultiplyAndAdd(0.5, gridOrigin);
            }
            
            public void GetFirstCenter(double[] point)
            {
                int LPlus = Level + 1;

                root.AddGridStep(LPlus);

                Buffer.BlockCopy(root.scaledSteps[LPlus], 0, point, 0, root.VectorSize);
                
                point.MultiplyAndAdd(0.5, gridOrigin);
            }

            public void CopyOrigin(double[] point, int offset = 0) => 
                Buffer.BlockCopy(gridOrigin, 0, point, offset, root.VectorSize);

            public void CopyGridStep(double[] step, int offset = 0) => 
                Buffer.BlockCopy(root.scaledSteps[Level], 0, step, offset, root.VectorSize);

            public void GetPlacement(double[] placement, int offset = 0)
            {
                Buffer.BlockCopy(gridOrigin, 0, placement, offset, root.VectorSize);
                offset += root.VectorSize;
                Buffer.BlockCopy(root.scaledSteps[Level], 0, placement, offset, root.VectorSize);
            }

            private object locker = new Object();

            /// <summary>
            /// Ini localised voxel {isFree = false , Level = 0, ancestor = null}; 
            /// the origin point is copied; 
            /// if the initializer is null then Data is set to default(T)
            /// </summary>
            public Voxel(AdaptiveGrid<T> root, int serial, double[] origin, Func<IVoxel, T> initializer = null, T iniValue = default(T))
            {
                this.root = root;
                SI = serial;

                isFree = false;
                ancestor = null;
                Level = 0;

                gridOrigin = new double[root.Dimension];
                Buffer.BlockCopy(origin, 0, gridOrigin, 0, root.VectorSize);

                Data = initializer == null ? iniValue : initializer(this);
            }

            /// <summary>
            /// Ini unlocalised voxel {isFree = true , Level = -1, ancestor = null, Data = default(T), gridOrigin = new double[root.Dimension]};
            /// </summary>
            public Voxel(AdaptiveGrid<T> root, int serial)
            {
                this.root = root;
                SI = serial;

                isFree = true;
                ancestor = null;
                Level = -1;

                gridOrigin = new double[root.Dimension];

                Data = default(T);
            }


            /// <summary>
            /// Sets data for content of this voxel and returns true if success
            /// </summary>
            /// <param name="mandatorily">If true then thread will be blocked until other threads will end up processing of this voxel</param>
            /// <param name="initializer">If null, then the iniValue will be used to initiate data of new voxels</param>
            /// <param name="iniValue"></param>
            /// <returns></returns>
            public bool SetContent(bool mandatorily, Func<IVoxel, T> initializer = null, T iniValue = default(T))
            {
                if (mandatorily) Monitor.Enter(locker);
                else
                {
                    // The checking of the state in advance gives tangible performance increase...
                    if (state == root.ProcessedState || !Monitor.TryEnter(locker)) return false;

                    // ...but in very rare cases it is possible that some thread will pass through
                    // the above check after another one has already set the state to processed, 
                    // completed the loop below and unlocked this voxel
                    if (state == root.ProcessedState)
                    {
                        Monitor.Exit(locker);
                        return false;
                    }
                    else state = root.ProcessedState;
                }

                //// It looks more correct than the above variant but it is slower
                //if (mandatorily)
                //{
                //    Monitor.Enter(locker);
                //}
                //else if (!Monitor.TryEnter(locker))
                //{
                //    return false;
                //}
                //else if (state == root.ProcessedState)
                //{
                //    Monitor.Exit(locker);
                //    return false;
                //}
                //else state = root.ProcessedState;

                try
                {
                    if (isFree || contentSI <= 0) return false;
                    
                    for (int serial = contentSI, stop = serial + root.tessMultp; serial < stop; serial++)
                    {
                        Voxel voxel = root.VStorage[serial];

                        voxel.Data = initializer == null ? iniValue : initializer(voxel);
                    }

                    return true;
                }
                finally
                {
                    Monitor.Exit(locker);
                }
            }

            /// <summary>
            /// Returns true only if this voxel has been successfully tessellated
            /// </summary>
            /// <param name="mandatorily">If true then thread will be blocked until other threads will end up processing of this voxel</param>
            /// <param name="initializer">If null, then the iniValue will be used to initiate data of new voxels</param>
            /// <param name="iniValue"></param>
            /// <returns></returns>
            public bool Tessellate(bool mandatorily, Func<IVoxel, T> initializer = null, T iniValue = default(T))
            {
                if (mandatorily) Monitor.Enter(locker);
                else
                {
                    // The checking of the state in advance gives tangible performance increase...
                    if (state == root.ProcessedState || !Monitor.TryEnter(locker)) return false;

                    // ...but in very rare cases it is possible that some thread will pass through
                    // the above check after another one has already set the state to processed, 
                    // completed the loop below and unlocked this voxel
                    if (state == root.ProcessedState)
                    {
                        Monitor.Exit(locker);
                        return false;
                    }
                    else state = root.ProcessedState;
                }

                try
                {
                    if (isFree || contentSI > 0 || !root.GetSerial(ref contentSI))
                        return false;

                    int dims = root.Dimension;
                    int nextLevel = Level + 1;

                    // check availability of step vector for the next level
                    root.AddScaledStep(nextLevel);

                    // initialize internal voxels
                    int[] index = new int[dims];
                    double[] Oi = new double[dims];

                    Buffer.BlockCopy(gridOrigin, 0, Oi, 0, root.VectorSize);

                    Voxel voxel;

                    for (int serial = contentSI, length = contentSI + root.tessMultp; serial < length; serial++)
                    {
                        voxel = root.VStorage[serial];

                        voxel.Level = nextLevel;
                        voxel.ancestor = this;
                        voxel.isFree = false;
                        voxel.state = !root.ProcessedState;
                        voxel.contentSI = -1;
                        
                        Buffer.BlockCopy(Oi, 0, voxel.gridOrigin, 0, root.VectorSize);

                        voxel.Data = initializer == null ? iniValue : initializer(voxel);

                        NextOrigin(index, Oi, nextLevel);
                    }

                    return true;
                }
                finally
                {
                    Monitor.Exit(locker);
                }
            }

            /// <summary>
            /// Returns true only if this voxel has been successfully tessellated
            /// </summary>
            /// <param name="mandatorily">If true then thread will be blocked until other threads will end up processing of this voxel</param>
            /// <param name="scan">Tessellation is performed if the predicate is true and all other conditions are met</param>
            /// <param name="initializer">If null, then the iniValue will be used to initiate data of new voxels</param>
            /// <param name="iniValue"></param>
            /// <returns></returns>
            public bool Tessellate(bool mandatorily, Predicate<IVoxel> scan, Func<IVoxel, T> initializer, T iniValue = default(T))
            {
                if (mandatorily) Monitor.Enter(locker);
                else
                {
                    // The checking of the state in advance gives tangible performance increase...
                    if (state == root.ProcessedState || !Monitor.TryEnter(locker)) return false;

                    // ...but in very rare cases it is possible that some thread will pass through
                    // the above check after another one has already set the state to processed, 
                    // completed the loop below and unlocked this voxel
                    if (state == root.ProcessedState)
                    {
                        Monitor.Exit(locker);
                        return false;
                    }
                    else state = root.ProcessedState;
                }

                try
                {
                    if (isFree || contentSI > 0 || !root.GetSerial(ref contentSI))
                        return false;

                    if (!scan(this))
                    {
                        root.ReleaseSerial(ref contentSI);
                        return false;
                    }

                    int dims = root.Dimension;
                    int nextLevel = Level + 1;

                    // check availability of step vector for the next level
                    root.AddScaledStep(nextLevel);

                    // initialize internal voxels
                    int[] index = new int[dims];
                    double[] Oi = new double[dims];

                    Buffer.BlockCopy(gridOrigin, 0, Oi, 0, root.VectorSize);

                    Voxel voxel;

                    for (int serial = contentSI, length = contentSI + root.tessMultp; serial < length; serial++)
                    {
                        voxel = root.VStorage[serial];

                        voxel.Level = nextLevel;
                        voxel.ancestor = this;
                        voxel.isFree = false;
                        voxel.state = !root.ProcessedState;
                        voxel.contentSI = -1;

                        Buffer.BlockCopy(Oi, 0, voxel.gridOrigin, 0, root.VectorSize);

                        voxel.Data = initializer == null ? iniValue : initializer(voxel);

                        NextOrigin(index, Oi, nextLevel);
                    }

                    return true;
                }
                finally
                {
                    Monitor.Exit(locker);
                }
            }

            /// <summary>
            /// Returns true only if this voxel has been successfully tessellated;
            /// this method is useful if the scan is responsible for the new voxels data
            /// </summary>
            /// <param name="mandatorily">If true then thread will be blocked until other threads will end up processing of this voxel</param>
            /// <param name="scan">Tessellation is performed if the predicate is true and all other conditions are met</param>
            /// <returns></returns>
            public bool Tessellate(bool mandatorily, Predicate<IVoxel> scan)
            {
                if (mandatorily) Monitor.Enter(locker);
                else
                {
                    // The checking of the state in advance gives tangible performance increase...
                    if (state == root.ProcessedState || !Monitor.TryEnter(locker)) return false;

                    // ...but in very rare cases it is possible that some thread will pass through
                    // the above check after another one has already set the state to processed, 
                    // completed the loop below and unlocked this voxel
                    if (state == root.ProcessedState)
                    {
                        Monitor.Exit(locker);
                        return false;
                    }
                    else state = root.ProcessedState;
                }

                try
                {
                    if (isFree || contentSI > 0 || !root.GetSerial(ref contentSI))
                        return false;

                    if (!scan(this))
                    {
                        root.ReleaseSerial(ref contentSI);
                        return false;
                    }

                    int dims = root.Dimension;
                    int nextLevel = Level + 1;

                    // check availability of step vector for the next level
                    root.AddScaledStep(nextLevel);

                    // initialize internal voxels
                    int[] index = new int[dims];
                    double[] Oi = new double[dims];

                    Buffer.BlockCopy(gridOrigin, 0, Oi, 0, root.VectorSize);

                    Voxel voxel;

                    for (int serial = contentSI, length = contentSI + root.tessMultp; serial < length; serial++)
                    {
                        voxel = root.VStorage[serial];

                        voxel.Level = nextLevel;
                        voxel.ancestor = this;
                        voxel.isFree = false;
                        voxel.state = !root.ProcessedState;
                        voxel.contentSI = -1;

                        Buffer.BlockCopy(Oi, 0, voxel.gridOrigin, 0, root.VectorSize);

                        NextOrigin(index, Oi, nextLevel);
                    }

                    return true;
                }
                finally
                {
                    Monitor.Exit(locker);
                }
            }

            /// <summary>
            /// Eliminates information about the content of this voxel
            /// by non-recursive traversal of the whole voxel subtree
            /// </summary>
            /// <param name="mandatorily">If true then thread will be blocked until other threads will end up processing of this voxel</param>
            /// <param name="initializer">If null, then the iniValue will be used to set data of this voxel</param>
            /// <param name="iniValue"></param>
            /// <returns></returns>
            public bool Merge(bool mandatorily, Func<IVoxel, T> initializer, T iniValue = default(T))
            {
                if (mandatorily) Monitor.Enter(locker);
                else
                {
                    // The checking of the state in advance gives tangible performance increase...
                    if (state == root.ProcessedState || !Monitor.TryEnter(locker)) return false;

                    // ...but in very rare cases it is possible that some thread will pass through
                    // the above check after another one has already set the state to processed, 
                    // completed the loop below and unlocked this voxel
                    if (state == root.ProcessedState)
                    {
                        Monitor.Exit(locker);
                        return false;
                    }
                    else state = root.ProcessedState;
                }

                //Stack<object> locksBunch = null; // Deep Lock

                try
                {
                    ////### Debug:Voxel.state
                    //if (state != root.ProcessedState) root.errNum++;
                    //else root.wellNum++;

                    if (isFree || contentSI <= 0) return false;

                    //locksBunch = new Stack<object>(root.scaledSteps.Count - Level); // Deep Lock
                    //locksBunch.Push(locker); // Deep Lock

                    Voxel parentV = this;
                    Voxel childV = root.VStorage[contentSI];
                    bool goUp = false;

                    // voxel-tree traversal loop
                    while (!goUp)
                    {
                        //Monitor.Enter(childV.locker); // Deep Lock

                        if (childV.isFree)
                        {
                            //Monitor.Exit(childV.locker); // Deep Lock
                        }
                        else if (childV.contentSI <= 0)
                        {
                            childV.isFree = true;
                            childV.ancestor = null;

                            //Monitor.Exit(childV.locker); // Deep Lock
                        }
                        else // childV has content
                        {
                            //locksBunch.Push(childV.locker); // Deep Lock

                            parentV = childV;
                            childV = root.VStorage[parentV.contentSI];

                            continue; // dive to inner voxels
                        }

                        goUp = true;

                        while (goUp && childV != this)
                        {
                            int serial = childV.SI + 1;
                            int S0 = parentV.contentSI;

                            if (serial - parentV.contentSI < root.tessMultp)
                            {
                                childV = root.VStorage[serial];
                                goUp = false;

                                break; // continue at current level
                            }
                            else // there are no more voxels at current level
                            {
                                root.ReleaseSerial(ref parentV.contentSI);
                                root.DecreaseLevelCount(childV.Level);
                                
                                if (parentV != this) parentV.isFree = true;

                                // emerge to upper level
                                childV = parentV;
                                parentV = parentV.ancestor;

                                //Monitor.Exit(locksBunch.Pop()); // Deep Lock
                            }
                        }
                    } // end of voxel-tree traversal loop

                    Data = initializer == null ? iniValue : initializer(this);

                    return true;
                }
                finally
                {
                    Monitor.Exit(locker);
                    //if (locksBunch == null) Monitor.Exit(locker); // Deep Lock
                    //else foreach (object L in locksBunch) Monitor.Exit(L); // Deep Lock
                }
            }

            /// <summary>
            /// Eliminates information about the content of this voxel
            /// by non-recursive traversal of the whole voxel subtree
            /// </summary>
            /// <param name="mandatorily">If true then thread will be blocked until other threads will end up processing of this voxel</param>
            /// <returns></returns>
            public bool Merge(bool mandatorily)
            {
                if (mandatorily) Monitor.Enter(locker);
                else
                {
                    // The checking of the state in advance gives tangible performance increase...
                    if (state == root.ProcessedState || !Monitor.TryEnter(locker)) return false;

                    // ...but in very rare cases it is possible that some thread will pass through
                    // the above check after another one has already set the state to processed, 
                    // completed the loop below and unlocked this voxel
                    if (state == root.ProcessedState)
                    {
                        Monitor.Exit(locker);
                        return false;
                    }
                    else state = root.ProcessedState;
                }

                //Stack<object> locksBunch = null; // Deep Lock

                try
                {
                    ////### Debug:Voxel.state
                    //if (state != root.ProcessedState) root.errNum++;
                    //else root.wellNum++;

                    if (isFree || contentSI <= 0) return false;

                    //locksBunch = new Stack<object>(root.scaledSteps.Count - Level); // Deep Lock
                    //locksBunch.Push(locker); // Deep Lock

                    Voxel parentV = this;
                    Voxel childV = root.VStorage[contentSI];
                    bool goUp = false;

                    // voxel-tree traversal loop
                    while (!goUp)
                    {
                        //Monitor.Enter(childV.locker); // Deep Lock

                        if (childV.isFree)
                        {
                            //Monitor.Exit(childV.locker); // Deep Lock
                        }
                        else if (childV.contentSI <= 0)
                        {
                            childV.isFree = true;
                            childV.ancestor = null;

                            //Monitor.Exit(childV.locker); // Deep Lock
                        }
                        else // childV has content
                        {
                            //locksBunch.Push(childV.locker); // Deep Lock

                            parentV = childV;
                            childV = root.VStorage[parentV.contentSI];

                            continue; // dive to inner voxels
                        }

                        goUp = true;

                        while (goUp && childV != this)
                        {
                            int serial = childV.SI + 1;
                            int S0 = parentV.contentSI;

                            if (serial - parentV.contentSI < root.tessMultp)
                            {
                                childV = root.VStorage[serial];
                                goUp = false;

                                break; // continue at current level
                            }
                            else // there are no more voxels at current level
                            {
                                root.ReleaseSerial(ref parentV.contentSI);
                                root.DecreaseLevelCount(childV.Level);

                                if (parentV != this) parentV.isFree = true;

                                // emerge to upper level
                                childV = parentV;
                                parentV = parentV.ancestor;

                                //Monitor.Exit(locksBunch.Pop()); // Deep Lock
                            }
                        }
                    } // end of voxel-tree traversal loop
                    
                    return true;
                }
                finally
                {
                    Monitor.Exit(locker);
                    //if (locksBunch == null) Monitor.Exit(locker); // Deep Lock
                    //else foreach (object L in locksBunch) Monitor.Exit(L); // Deep Lock
                }
            }

            /// <summary>
            /// Eliminates information about the content of this voxel
            /// by recursive traversal of the whole voxel subtree
            /// </summary>
            /// <param name="mandatorily">If true then thread will be blocked until other threads will end up processing of this voxel</param>
            /// <param name="initializer">If null, then the iniValue will be used to set data of this voxel</param>
            /// <param name="iniValue"></param>
            /// <returns></returns>
            public bool Merge_R(bool mandatorily, Func<IVoxel, T> initializer = null, T iniValue = default(T))
            {
                if (mandatorily) Monitor.Enter(locker);
                else
                {
                    // The checking of the state in advance gives tangible performance increase...
                    if (state == root.ProcessedState || !Monitor.TryEnter(locker)) return false;

                    // ...but in very rare cases it is possible that some thread will pass through
                    // the above check after another one has already set the state to processed, 
                    // completed the loop below and unlocked this voxel
                    if (state == root.ProcessedState)
                    {
                        Monitor.Exit(locker);
                        return false;
                    }
                    else state = root.ProcessedState;
                }

                try
                {
                    if (isFree || contentSI <= 0) return false;

                    for (int serial = contentSI, length = contentSI + root.tessMultp; serial < length; serial++)
                    {
                        root.VStorage[serial].Prune_R();
                    }

                    root.ReleaseSerial(ref contentSI);
                    root.DecreaseLevelCount(Level + 1);

                    Data = initializer == null ? iniValue : initializer(this);

                    return true;
                }
                finally
                {
                    Monitor.Exit(locker);
                }
            }

            /// <summary>
            /// Recursive pruning
            /// </summary>
            private void Prune_R()
            {
                lock (locker)
                {
                    if (isFree) return;

                    if (contentSI > 0)
                    {
                        for (int serial = contentSI, length = contentSI + root.tessMultp; serial < length; serial++)
                        {
                            root.VStorage[serial].Prune_R();
                        }

                        root.ReleaseSerial(ref contentSI);
                        root.DecreaseLevelCount(Level + 1);
                    }

                    isFree = true;
                    ancestor = null;
                }
            }

            /// <summary>
            /// Performs traversal of the point over the origins of direct childs of the given voxel;
            /// after passing the last child voxel the point is positioned 
            /// at the origin of this voxel, index is reset to zero and function returns false
            /// </summary>
            /// <param name="index">Current index-vector; will be incremented</param>
            /// <param name="point">Current child voxel origin; will be shifted by the grid step of this voxel</param>
            /// <param name="nextLev">The level of children of this voxel</param>
            /// <returns></returns>
            private bool NextOrigin(int[] index, double[] point, int nextLev)
            {
                int i = 0;

                while (i < root.Dimension)
                {
                    if (++index[i] < root.tessFactors[i])
                    {
                        point[i] += root.scaledSteps[nextLev][i];
                        return true;
                    }
                    else
                    {
                        index[i] = 0;
                        point[i] = gridOrigin[i];

                        i++;
                    }
                }

                return false;
            }

            /// <summary>
            /// Performs traversal of the point over the origins of direct childs of the given voxel;
            /// after passing the last child voxel the point is positioned 
            /// at the origin of this voxel, index is reset to zero and function returns false
            /// </summary>
            /// <param name="index">Current index-vector; will be incremented</param>
            /// <param name="point">Current child voxel origin; will be shifted by the grid step of this voxel</param>
            /// <returns></returns>
            public bool NextOrigin(int[] index, double[] point)
            {
                int i = 0;
                int levelPlus = Level + 1;

                while (i < root.Dimension)
                {
                    if (++index[i] < root.tessFactors[i])
                    {
                        point[i] += root.scaledSteps[levelPlus][i];
                        return true;
                    }
                    else
                    {
                        index[i] = 0;
                        point[i] = gridOrigin[i];

                        i++;
                    }
                }

                return false;
            }

            /// <summary>
            /// Performs traversal of the point over the centers of direct childs of the given voxel;
            /// after passing the last child voxel the point is positioned at the center
            /// of the first child of this voxel, index is reset to zero and function returns false
            /// </summary>
            /// <param name="index">Current index-vector; will be incremented</param>
            /// <param name="point">Current child voxel center; will be shifted by the grid step of this voxel</param>
            /// <returns></returns>
            public bool NextCenter(int[] index, double[] point)
            {
                int i = 0;
                int levelPlus = Level + 1;

                while (i < root.Dimension)
                {
                    if (++index[i] < root.tessFactors[i])
                    {
                        point[i] += root.scaledSteps[levelPlus][i];
                        return true;
                    }
                    else
                    {
                        index[i] = 0;
                        point[i] = gridOrigin[i];
                        point[i] += 0.5 * root.scaledSteps[levelPlus][i];

                        i++;
                    }
                }

                return false;
            }
        }


        public readonly int Dimension; // dimension of the space the grid lies in

        private const int MaxCacheLength = 250000;
        public readonly int VectorSize; // in bytes
        public readonly int TessArrSize; // in bytes

        private Voxel[] VStorage; // contains all voxels of this grid
        private int[] indexCache; // retains last available free slot indexes
        private int cacheEndex; // length of the indexCache array minus one
        private int cacheInd; // the number of cached indexes minus one
        private int freeNum; // the number of remaining vacancies

        private double[] rootOrigin;
        private double[] rootStep;
        private int[] rootLengths; // max = step * regularLengths[coordIndex];
        public readonly int RootsNumber; // the number of voxels in the root-grid

        private List<double[]> scaledSteps; // grid steps, precalculated for several scales
        private List<int> levelCount; // the number of voxels at levels
        private int totalUsed; // total voxels is used

        private int[] tessFactors; // tessellation edge factors
        private readonly int tessMultp; // number of new voxels resulting of the Tessellation
        private int[] dimMultps; // for conversion from spatial index to serial index
        private int[] dimMultpsRoot;

        public IList<IVoxel> InternalArray { get { return Array.AsReadOnly<IVoxel>(VStorage); } }

        public IList<double> GridStep(int level) => 
            Array.AsReadOnly(scaledSteps[level]);

        public void CopyGridStep(int level, double[] step, int offset = 0) => 
            Buffer.BlockCopy(scaledSteps[level], 0, step, offset, VectorSize);

        public IList<int> TesselationFactors { get { return Array.AsReadOnly(tessFactors); } }
        public int TessellationMultp { get { return tessMultp; } }
        public int ActiveCount { get { return totalUsed; } }
        public IList<int> LevelsCounts { get { return levelCount.AsReadOnly(); } }
        public int LevelsNumber { get { return levelCount.Count; } }
        public bool IsFull { get { return freeNum == 0; } }

        /// <summary>
        /// If value of a voxel state property is equal to the ProcessedState, then such voxel
        /// is considered as processed and voxel methods with "mandatorily" parameter set to false,
        /// return immediately
        /// </summary>
        public bool ProcessedState { get; set; }

        /// <summary>
        /// This method is an O(n) operation, where n is the InternalArray.Count
        /// </summary>
        /// <param name="state"></param>
        public void UnifyStates(bool state) { for (int i = 0; i < VStorage.Length; i++) VStorage[i].state = state; }

        /// <summary>
        /// Sets the given state to the root voxels
        /// </summary>
        /// <param name="state"></param>
        public void SetRootStates(bool state) { for (int i = 0; i < RootsNumber; i++) VStorage[i].state = state; }

        /// <summary>
        /// All voxels are stored in the internal array,
        /// Total number of voxels is defined as follows: rootNum + tessNum * tessMult,
        /// where rootNum is the number of voxels in the root-grid
        /// and tessMult is the tessellation multiplier;
        /// Shape and tessellation arrays are copied
        /// </summary>
        /// <param name="tessNum">Max number of tessellations</param>
        /// <param name="Shape">Root-grid shape; Shape[coordIndex, :] = {min, max, num}; 
        /// where (max > min AND num > 0) should be true for every coordIndex</param>
        /// <param name="tessellation">Tessellation edge factors; if null then tess. will correspond to the Shape</param>
        /// <param name="tessDepth">Sets the number of preliminary calculations of the scaled grid steps; -1 for auto setting</param>
        /// <param name="initializer">If null, then the iniValue will be used to initiate data of new voxels</param>
        /// <param name="iniValue"></param>
        /// <param name="cacheLength">Sets length of index cache array; less than 1 for auto setting</param>
        public AdaptiveGrid(int tessNum, double[,] Shape, int[] tessellation = null, int tessDepth = -1, 
                            Func<IVoxel, T> initializer = null, T iniValue = default(T), int cacheLength = -1)
        {
            if (tessNum <= 0)
                throw new ArgumentException("tessNum should be greater than zero");

            if (!CheckShape(Shape))
                throw new ArgumentException("The Shape array is invalid");

            Dimension = Shape.GetLength(0);

            if (tessellation != null && (tessellation.Length != Dimension || tessellation.Any(k => k <= 0)))
                throw new ArgumentException("The tessellation array is invalid");
            
            VectorSize = Dimension * sizeof(double);
            TessArrSize = Dimension * sizeof(int);
            
            rootStep = new double[Dimension];
            rootOrigin = new double[Dimension];
            rootLengths = new int[Dimension];
            
            RootsNumber = 1;
            
            // --- Convert the Shape array and calculate total number of voxels in the corresponding regular grid -----------
            for (int i = 0; i < Dimension; i++)
            {
                // convertion from {min, max, num} to num and {min, step}
                rootLengths[i] = (int)Shape[i, 2];
                rootOrigin[i] = Shape[i, 0];
                rootStep[i] = (Shape[i, 1] - Shape[i, 0]) / rootLengths[i];

                RootsNumber *= rootLengths[i];
            }

            // --- Set default tessellation factors -------------------------------
            if (tessellation != null)
            {
                tessFactors = new int[Dimension];
                Buffer.BlockCopy(tessellation, 0, tessFactors, 0, TessArrSize);

                tessMultp = tessFactors.Aggregate((num, k) => num * k);
            }
            else
            {
                tessFactors = rootLengths;
                tessMultp = RootsNumber;
            }

            // calc dimension multipliers
            dimMultps = new int[Dimension];
            dimMultpsRoot = new int[Dimension];
            dimMultps[0] = 1;
            dimMultpsRoot[0] = 1;

            for (int i = 1, k = 0; i < Dimension; i++, k++)
            {
                dimMultps[i] = tessFactors[k] * dimMultps[k];
                dimMultpsRoot[i] = rootLengths[k] * dimMultpsRoot[k];
            }

            // --- Preliminary calculations of the scaled grid steps --------------
            if (tessDepth < 0)
                tessDepth = tessNum / (2 * RootsNumber);
            else
                tessDepth = Math.Min(tessNum, tessDepth);

            scaledSteps = new List<double[]>(++tessDepth); // +1 for the root
            scaledSteps.Add(rootStep);

            levelCount = new List<int>(tessDepth) { RootsNumber };
            totalUsed = RootsNumber;

            double[] lastStep = rootStep, nextStep;
            
            for (int n = 1; n < tessDepth; n++)
            {
                nextStep = new double[Dimension];

                for (int i = 0; i < Dimension; i++) nextStep[i] = lastStep[i] / tessFactors[i];

                scaledSteps.Add(nextStep);
                lastStep = nextStep;
            }

            // --- Initialization of voxels --------------------------------------
            int capacity = RootsNumber + tessNum * tessMultp;

            VStorage = new Voxel[capacity];

            int[] index = new int[Dimension];
            double[] Oi = new double[Dimension];

            Buffer.BlockCopy(rootOrigin, 0, Oi, 0, VectorSize);

            // ini root voxels
            for (int serial = 0; serial < RootsNumber; serial++)
            {
                VStorage[serial] = new Voxel(this, serial, Oi, initializer, iniValue);
                NextPoint(index, Oi);
            }

            // ini remaining voxels
            for (int serial = RootsNumber; serial < capacity; serial++)
            {
                VStorage[serial] = new Voxel(this, serial);
            }

            // --- Set indexCache ----------------------------------------------------
            freeNum = tessNum;

            if (cacheLength <= 0) cacheLength = Math.Min(capacity - RootsNumber, MaxCacheLength);
            else cacheLength = Math.Min(capacity - RootsNumber, cacheLength);

            indexCache = new int[cacheLength];
            indexCache[0] = RootsNumber;
            cacheEndex = cacheLength - 1;
            cacheInd = 0;
        }

        /// <summary>
        /// Checking of the Shape array: Shape[coordIndex, :] = {min, max, num},
        /// returns true if for every coordIndex the statement (max > min && num > 0) is true, otherwise false
        /// </summary>
        /// <param name="Shape"></param>
        /// <returns></returns>
        public bool CheckShape(double[,] Shape)
        {
            if (Shape.GetLength(1) != 3) return false;

            int length = Shape.GetLength(0);

            for (int i = 0; i < length; i++)
            {
                if (Shape[i, 0] >= Shape[i, 1] || Shape[i, 2] <= 0) return false;
            }

            return true;
        }

        /// <summary>
        /// Calculates grid step at the given level if needed;
        /// Is used for tessellation
        /// </summary>
        /// <param name="level"></param>
        private void AddScaledStep(int level)
        {
            lock (levelCount)
            {
                totalUsed += tessMultp;

                if (level < scaledSteps.Count) // then just change levelCount
                {
                    if (level < levelCount.Count)
                    {
                        levelCount[level] += tessMultp;
                    }
                    else levelCount.Add(tessMultp);

                    return;
                }

                // calc scaled steps for the new level
                double[] lastStep = scaledSteps[level - 1];
                double[] nextStep = new double[Dimension];
                
                for (int i = 0; i < Dimension; i++) nextStep[i] = lastStep[i] / tessFactors[i];

                scaledSteps.Add(nextStep);
                levelCount.Add(tessMultp);
            }
        }

        /// <summary>
        /// Calculates and adds the grid step at the level following the last one
        /// at which precalculated steps are available
        /// </summary>
        public void AddGridStep()
        {
            lock (levelCount)
            {
                double[] lastStep = scaledSteps[scaledSteps.Count - 1];
                double[] nextStep = new double[Dimension];

                for (int i = 0; i < Dimension; i++) nextStep[i] = lastStep[i] / tessFactors[i];

                scaledSteps.Add(nextStep);
            }
        }

        /// <summary>
        /// If the given level less than or equal to the last level at which precalculated steps are available
        /// then immediately returns true;
        /// If the given level follows the last one, then calculates grid step at the given level and returns true;
        /// Otherwise immediately returns false;
        /// </summary>
        /// <param name="level"></param>
        public bool AddGridStep(int level)
        {
            lock (levelCount)
            {
                if (level < scaledSteps.Count)
                    return true;
                else if (level > scaledSteps.Count)
                    return false;

                double[] lastStep = scaledSteps[scaledSteps.Count - 1];
                double[] nextStep = new double[Dimension];

                for (int i = 0; i < Dimension; i++) nextStep[i] = lastStep[i] / tessFactors[i];

                scaledSteps.Add(nextStep);

                return true;
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="level"></param>
        private void DecreaseLevelCount(int level)
        {
            lock (levelCount)
            {
                levelCount[level] -= tessMultp;
                totalUsed -= tessMultp;

                if (levelCount[level] == 0) levelCount.RemoveAt(level);
            }
        }

        /// <summary>
        /// Searches for free slot in VStorage and occupies it;
        /// Returns false if there are no free slots
        /// </summary>
        /// <param name="serial">Negative values indicate that serial has not been set yet</param>
        /// <returns></returns>
        private bool GetSerial(ref int serial)
        {
            lock (indexCache)
            {
                if (freeNum == 0) return false;

                if (cacheInd == 0) // then use cached index
                {
                    serial = indexCache[cacheInd];
                    VStorage[serial].isFree = false;
                    freeNum--;

                    // and check the next slot
                    int nexti = serial + tessMultp;

                    if (nexti < VStorage.Length && VStorage[nexti].isFree)
                    {
                        indexCache[cacheInd] = nexti;
                    }
                    else cacheInd--;

                    return true;
                }
                else if (cacheInd > 0) // then use cached index
                {
                    serial = indexCache[cacheInd];
                    VStorage[serial].isFree = false;
                    cacheInd--;
                    freeNum--;

                    return true;
                }
                else // search vacancies directly in VStorage
                {
                    for (int i = RootsNumber; i < VStorage.Length; i += tessMultp)
                    {
                        if (VStorage[i].isFree)
                        {
                            serial = i;
                            VStorage[i].isFree = false;
                            freeNum--;
                            return true;
                        }
                    }
                }
            }

            return false;
        }
        
        private void ReleaseSerial(ref int serial)
        {
            lock (indexCache)
            {
                if (cacheInd < cacheEndex) indexCache[++cacheInd] = serial;

                VStorage[serial].isFree = true;
                serial = -1;
                freeNum++;
            }
        }

        /// <summary>
        /// Performs traversal of the point over the root grid;
        /// after passing the last voxel the point is positioned 
        /// at the root grid origin, index is reset to zero and function returns false
        /// </summary>
        /// <param name="index">Current index-vector; will be incremented</param>
        /// <param name="point">Current voxel origin; will be shifted by the root grid step</param>
        /// <returns></returns>
        private bool NextPoint(int[] index, double[] point)
        {
            int i = 0;

            while (i < Dimension)
            {
                if (++index[i] < rootLengths[i])
                {
                    point[i] += rootStep[i];
                    return true;
                }
                else
                {
                    index[i] = 0;
                    point[i] = rootOrigin[i];

                    i++;
                }
            }

            return false;
        }

        /// <summary>
        /// Performs traversal of the point over the grid at the given level;
        /// after passing the last index the point is positioned 
        /// at the given origin, index is reset to zero and function returns false
        /// </summary>
        /// <param name="index">Current index-vector; will be incremented</param>
        /// <param name="point">Current point; will be shifted by the grid step</param>
        /// <param name="origin">Defines initial point of the traversal</param>
        /// <param name="level">Defines grid step to shift the point</param>
        /// <returns></returns>
        public bool NextOrigin(int[] index, double[] point, IList<double> origin, int level)
        {
            int i = 0;

            while (i < Dimension)
            {
                if (++index[i] < tessFactors[i])
                {
                    point[i] += scaledSteps[level][i];
                    return true;
                }
                else
                {
                    index[i] = 0;
                    point[i] = origin[i];

                    i++;
                }
            }

            return false;
        }

        /// <summary>
        /// Performs traversal of the point over the origins of direct childs of the given voxel;
        /// after passing the last index the point is positioned 
        /// at the given origin, index is reset to zero and function returns false
        /// </summary>
        /// <param name="index"></param>
        /// <param name="point"></param>
        /// <param name="voxel"></param>
        /// <returns></returns>
        public bool NextOrigin(int[] index, double[] point, IVoxel voxel)
        {
            int i = 0;
            int level = voxel.TessLevel + 1;

            while (i < Dimension)
            {
                if (++index[i] < tessFactors[i])
                {
                    point[i] += scaledSteps[level][i];
                    return true;
                }
                else
                {
                    index[i] = 0;
                    point[i] = voxel.Origin[i];

                    i++;
                }
            }

            return false;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="serials"></param>
        /// <param name="ivoxel"></param>
        /// <returns></returns>
        public bool GetVoxel(int[] serials, out IVoxel ivoxel)
        {
            ivoxel = null;
            Voxel V;
            int index = serials[0];

            if (index < RootsNumber) V = VStorage[index];
            else return false;

            for (int i = 1; i < serials.Length; i++)
            {
                if (V.contentSI < 0)
                {
                    ivoxel = V;
                    return false;
                }

                index = serials[i];

                if (index >= tessMultp) return false;

                index += V.contentSI;
                V = VStorage[index];
            }

            ivoxel = V;
            return true;
        }

        /// <summary>
        /// Traverses currV parameter over the voxels located at the same level;
        /// If there are no more voxels at currV level following the current voxel currV, 
        /// the method returns false and currV is not changed;
        /// Non-recursive method 
        /// </summary>
        /// <param name="currV">Current voxel, will be set to the next voxel at the same level</param>
        /// <returns></returns>
        public bool NextAtLevel(ref IVoxel currV)
        {
            Voxel V = VStorage[currV.SerialIndex];
            int level = V.Level; // traversal level
            int serial;
            bool atLevel = true;

            while (true)
            {
                serial = V.SI;
                serial++;
                
                if (V.Level == 0)
                {
                    if (serial >= RootsNumber) return false; // all voxels has been traversed
                }
                else if (serial - V.ancestor.contentSI >= tessMultp)
                {
                    V = V.ancestor;
                    atLevel = false;

                    continue;
                }

                V = VStorage[serial];

                if (atLevel)
                {
                    currV = V;
                    return true;
                }

                // else try to dive to the traversal level
                while (V.contentSI > 0)
                {
                    V = VStorage[V.contentSI];

                    if (V.Level >= level)
                    {
                        currV = V;
                        return true;
                    }
                }
            }
        }

        /// <summary>
        /// Traverses currV parameter over the voxels located at the same level within
        /// the same ancestor specfied by the index ancestorIndex;
        /// If there are no more voxels at currV level following the current voxel currV, 
        /// the method returns false;
        /// No verfication is performed that voxel with index ancestorIndex contains given currV;
        /// Non-recursive method 
        /// </summary>
        /// <param name="ancestorIndex">Index of ancestor to traverse its inner voxels</param>
        /// <param name="currV">Current voxel, will be set to the next voxel at the same level</param>
        /// <returns></returns>
        public bool NextAtLevel(int ancestorIndex, ref IVoxel currV)
        {
            Voxel V = VStorage[currV.SerialIndex];
            int ancestorLevel = VStorage[ancestorIndex].Level;
            int level = V.Level; // traversal level
            int serial;
            bool atLevel = true;

            while (V.Level > ancestorLevel)
            {
                serial = V.SI;
                serial++;

                if (serial - V.ancestor.contentSI >= tessMultp)
                {
                    V = V.ancestor;
                    atLevel = false;

                    continue;
                }

                V = VStorage[serial];

                if (atLevel)
                {
                    currV = V;
                    return true;
                }

                // else try to dive to the traversal level
                while (V.contentSI > 0)
                {
                    V = VStorage[V.contentSI];

                    if (V.Level >= level)
                    {
                        currV = V;
                        return true;
                    }
                }
            }

            return false;
        }

        /// <summary>
        /// Traverses currV parameter over the voxels located at the same level;
        /// If there are not enough voxels to accomplish the stride
        /// the method returns false;
        /// </summary>
        /// <param name="stride">At the root level it defines the number of root voxels to skip;
        /// At all other levels it defines the number of parent voxel (located one level above 
        /// of the currV level) which should be skipped</param>
        /// <param name="currV">Current voxel, will be set to the next voxel at the same level</param>
        /// <returns></returns>
        public bool SkipNextAtLevel(int stride, ref IVoxel currV)
        {
            if (currV.TessLevel == 0)
            {
                int serial = currV.SerialIndex + stride;

                if (serial >= RootsNumber) return false;

                currV = VStorage[serial];
                return true;
            }

            Voxel V = VStorage[currV.Ancestor.SerialIndex];
            int level = V.Level; // traversal level
            bool atLevel = true;
            
            while (true)
            {
                int serial = V.SI;
                serial++;

                if (V.Level == 0)
                {
                    if (serial >= RootsNumber) return false; // all voxels has been traversed
                }
                else if (serial - V.ancestor.contentSI >= tessMultp)
                {
                    V = V.ancestor;
                    atLevel = false;

                    continue;
                }

                V = VStorage[serial];

                if (atLevel)
                {
                    if (V.contentSI > 0 && stride-- == 0)
                    {
                        currV = VStorage[V.contentSI];
                        return true;
                    }
                    else continue;
                }

                // else try to dive to the traversal level
                while (V.contentSI > 0)
                {
                    V = VStorage[V.contentSI];

                    if (V.Level >= level)
                    {
                        if (V.contentSI > 0 && stride-- == 0)
                        {
                            currV = VStorage[V.contentSI];
                            return true;
                        }
                        else
                        {
                            atLevel = true;
                            break;
                        }
                    }
                }
            }
        }

        /// <summary>
        /// Traverses currV parameter over innermost voxels of the grid;
        /// If there are no more innermost voxels following the current voxel currV, 
        /// the method returns false and currV is set equal to the last root-voxel;
        /// Non-recursive method
        /// </summary>
        /// <param name="currV">Current voxel should be innermost for proper traversal;
        /// will be set to the next innermost voxel</param>
        /// <returns></returns>
        public bool NextInnermost(ref IVoxel currV)
        {
            int serial, S0;
            
            while (currV.TessLevel > 0)
            {
                serial = currV.SerialIndex + 1;
                S0 = currV.Ancestor.ContentSI;

                if (serial - S0 < tessMultp) // then go to the next voxel at the same level
                {
                    currV = VStorage[serial];

                    // dive to innermost voxel
                    while (currV.ContentSI > 0) currV = VStorage[currV.ContentSI];

                    return true;
                }
                else currV = currV.Ancestor;
            }

            // at the root level
            serial = currV.SerialIndex + 1;

            if (serial < RootsNumber) // then go to the next voxel at the root level
            {
                currV = VStorage[serial];

                // dive to innermost voxel
                while (currV.ContentSI > 0) currV = VStorage[currV.ContentSI];

                return true;
            }
            
            return false; // all innermost voxels has been traversed
        }

        /// <summary>
        /// If the given ref voxel contains voxels at the given level,
        /// then the ref voxel is set to the first voxel at the level
        /// and method returns true; otherwise ref voxel is not changed
        /// and method returns false
        /// </summary>
        /// <param name="level"></param>
        /// <param name="voxel"></param>
        /// <returns></returns>
        private bool GetAtLevel(int level, ref Voxel voxel)
        {
            if (level == voxel.Level) return true;
            else if (level < voxel.Level) return false;
            else if (voxel.contentSI < 0) return false;

            Voxel V = VStorage[voxel.contentSI];

            bool atLevel = true;

            while (atLevel || V.Level > voxel.Level)
            {
                if (level == V.Level)
                {
                    voxel = V;
                    return true;
                }

                if (atLevel && V.contentSI > 0)
                {
                    V = VStorage[V.contentSI];
                    continue;
                }

                int serial = V.SI + 1;

                if (serial - V.ancestor.contentSI >= tessMultp)
                {
                    V = V.ancestor;
                    atLevel = false;
                    continue;
                }

                V = VStorage[serial];
                atLevel = true;
            }

            return false;
        }

        /// <summary>
        /// If the given ivoxel contains voxels at the given level,
        /// then the ivoxel is set to the first voxel at the level
        /// and method returns true; otherwise ivoxel is not changed
        /// and method returns false
        /// </summary>
        /// <param name="level"></param>
        /// <param name="ivoxel"></param>
        /// <returns></returns>
        public bool GetAtLevel(int level, ref IVoxel ivoxel)
        {
            if (level == ivoxel.TessLevel) return true;
            else if (level < ivoxel.TessLevel) return false;
            else if (ivoxel.ContentSI < 0) return false;

            Voxel V = VStorage[ivoxel.ContentSI];

            int ivLevel = ivoxel.TessLevel;
            bool atLevel = true;

            while (atLevel || V.Level > ivLevel)
            {
                if (level == V.Level)
                {
                    ivoxel = V;
                    return true;
                }

                if (atLevel && V.contentSI > 0)
                {
                    V = VStorage[V.contentSI];
                    continue;
                }

                int serial = V.SI + 1;

                if (serial - V.ancestor.contentSI >= tessMultp)
                {
                    V = V.ancestor;
                    atLevel = false;
                    continue;
                }

                V = VStorage[serial];
                atLevel = true;
            }

            return false;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="level"></param>
        /// <param name="ivoxel"></param>
        /// <returns></returns>
        public bool FirstAtLevel(int level, out IVoxel ivoxel)
        {
            if (level == 0)
            {
                ivoxel = VStorage[0];
                return true;
            }

            ivoxel = null;

            for (int i = 0; i < RootsNumber; i++)
            {
                var V = VStorage[i];

                if (GetAtLevel(level, ref V))
                {
                    ivoxel = V;
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="level"></param>
        /// <param name="ancestorIndex"></param>
        /// <param name="ivoxel"></param>
        /// <returns></returns>
        public bool FirstAtLevel(int level, int ancestorIndex, out IVoxel ivoxel)
        {
            ivoxel = null;

            var V = VStorage[ancestorIndex];

            if (GetAtLevel(level, ref V))
            {
                ivoxel = V;
                return true;
            }

            return false;
        }

        /// <summary>
        /// If point is within grid range and it is located within
        /// voxel of given level, then returns true and 
        /// ivoxel is set to this voxel;
        /// Non-recursive method
        /// <param name="level"></param>
        /// <param name="point"></param>
        /// <param name="ivoxel"></param>
        /// <returns></returns>
        public bool GetAtLevel(int level, double[] point, out IVoxel ivoxel)
        {
            ivoxel = null;

            // --- Start search from the root-voxel ------------------------------------
            double[] Oj = rootOrigin;
            double[] Sj = rootStep;

            int serial = 0;
            int iLevel = 0;

            int index;
            double delta;

            // define i-coordinate and accumulate serial
            for (int i = 0; i < Dimension; i++)
            {
                delta = point[i] - Oj[i];
                if (delta < 0) return false; // >>> point is outside the lower bound >>>

                index = (int)(delta / Sj[i]);
                if (index >= rootLengths[i]) return false; // >>> point is outside the upper bound >>>

                serial += index * dimMultpsRoot[i];
            }
            
            Voxel voxel = VStorage[serial];

            if (level == iLevel) { ivoxel = voxel;  return true; }

            // go to inner voxels
            serial = voxel.contentSI;

            // --- Continue search in internal voxels -----------------------------------
            while (serial > 0)
            {
                iLevel++;
                Oj = voxel.gridOrigin;
                Sj = scaledSteps[iLevel];

                // define 0-coordinate and accumulate serial
                serial += (int)((point[0] - Oj[0]) / Sj[0]);

                // define i-coordinate and accumulate serial
                for (int i = 1; i < Dimension; i++)
                {
                    serial += dimMultps[i] * (int)((point[i] - Oj[i]) / Sj[i]);
                }
                
                voxel = VStorage[serial];

                if (level == iLevel) { ivoxel = voxel; return true; }

                // go to inner voxels
                serial = voxel.contentSI;
            }
            
            return false;
        }

        /// <summary>
        /// Returns the first innermost voxel contained in the given ivoxel
        /// </summary>
        /// <param name="ivoxel"></param>
        /// <returns></returns>
        public IVoxel GetInnermost(IVoxel ivoxel)
        {
            if (ivoxel.ContentSI < 0) return ivoxel;

            Voxel V = VStorage[ivoxel.ContentSI];
            Voxel inmostV = V;

            int ivLevel = ivoxel.TessLevel;
            bool atLevel = true;

            while (atLevel || V.Level > ivLevel)
            {
                if (atLevel && V.contentSI > 0)
                {
                    V = VStorage[V.contentSI];

                    if (V.Level > inmostV.Level) inmostV = V;

                    continue;
                }

                int serial = V.SI + 1;

                if (serial - V.ancestor.contentSI >= tessMultp)
                {
                    V = V.ancestor;
                    atLevel = false;
                    continue;
                }

                V = VStorage[serial];
                atLevel = true;
            }

            return inmostV;
        }

        /// <summary>
        /// If point is within grid range, then returns true and 
        /// ivoxel is set to innermost voxel containing given point;
        /// Non-recursive method
        /// </summary>
        /// <param name="point"></param>
        /// <returns></returns>
        public bool GetInnermost(double[] point, out IVoxel ivoxel)
        {
            ivoxel = null;

            // --- Start search from the root-voxel ------------------------------------
            double[] Oj = rootOrigin;
            double[] Sj = rootStep;

            int serial = 0;
            int level = 0;

            int index;
            double delta;

            // define i-coordinate and accumulate serial
            for (int i = 0; i < Dimension; i++)
            {
                delta = point[i] - Oj[i];
                if (delta < 0) return false; // >>> point is outside the lower bound >>>

                index = (int)(delta / Sj[i]);
                if (index >= rootLengths[i]) return false; // >>> point is outside the upper bound >>>

                serial += index * dimMultpsRoot[i];
            }

            // go to inner voxels
            Voxel voxel = VStorage[serial];
            serial = voxel.contentSI;

            // --- Continue search in internal voxels -----------------------------------
            while (serial > 0)
            {
                level++;
                Oj = voxel.gridOrigin;
                Sj = scaledSteps[level];
                
                // define 0-coordinate and accumulate serial
                serial += (int)((point[0] - Oj[0]) / Sj[0]);

                // define i-coordinate and accumulate serial
                for (int i = 1; i < Dimension; i++)
                {
                    serial += dimMultps[i] * (int)((point[i] - Oj[i]) / Sj[i]);
                }

                // go to inner voxels
                voxel = VStorage[serial];
                serial = voxel.contentSI;
            }

            ivoxel = voxel;
            return true;
        }

        /// <summary>
        /// Tessellates innermost voxel containing given point only if it is not
        /// already tessellated and there is a free slot for the new voxels;
        /// returns true, if there was a tessellation
        /// </summary>
        /// <param name="point"></param>
        /// <param name="mandatorily">If true then thread will be blocked until other threads will end up processing of this voxel
        /// <param name="initializer">If null, then the iniValue will be used to initiate data of new voxels</param>
        /// <param name="iniValue"></param>
        /// <returns></returns>
        public bool Tessellate(double[] point, bool mandatorily = false, Func<IVoxel, T> initializer = null, T iniValue = default(T))
        {
            IVoxel voxel = null;

            if (GetInnermost(point, out voxel)) return voxel.Tessellate(mandatorily, initializer, iniValue);
            else return false;
        }

        /// <summary>
        /// Translates relative serial index to corresponding index-vector
        /// at the root level
        /// </summary>
        /// <param name="rsi"></param>
        /// <param name="indexVector"></param>
        private void DecomposeRootSI(int rsi, int[] indexVector)
        {
            int i = Dimension;

            do
            {
                --i;

                indexVector[i] = Math.DivRem(rsi, dimMultpsRoot[i], out rsi);
            }
            while (i > 1);

            indexVector[0] = rsi;
        }

        /// <summary>
        /// Translates relative serial index to corresponding index-vector
        /// at all levels except the root
        /// </summary>
        /// <param name="rsi"></param>
        /// <param name="indexVector"></param>
        private void DecomposeRSI(int rsi, int[] indexVector)
        {
            int i = Dimension;

            do
            {
                --i;

                indexVector[i] = Math.DivRem(rsi, dimMultps[i], out rsi);
            }
            while (i > 1);

            indexVector[0] = rsi;
        }

        /// <summary>
        /// Translates relative serial index to corresponding index-vector
        /// </summary>
        /// <param name="rsi"></param>
        /// <param name="indexVector"></param>
        /// <param name="dimMultps"></param>
        private static void DecomposeRSI(int rsi, int[] indexVector, int[] dimMultps)
        {
            int i = dimMultps.Length;

            do
            {
                --i;

                indexVector[i] = Math.DivRem(rsi, dimMultps[i], out rsi);
            }
            while (i > 1);

            indexVector[0] = rsi;
        }

        // buffers for the GetIncrements and GetSerialIncrements methods
        private int[] currIndexVector;
        private int[] prevIndexVector;

        /// <summary>
        /// Calculates vector increments between centers of voxels with one direct ancestor;
        /// Vector increments array indexing: [tip_index][origin_index][component_index]
        /// </summary>
        /// <param name="level">The level of voxels which centers are connected by the vector increments</param>
        /// <param name="increments">Array to storing results; should be initialized</param>
        /// <returns></returns>
        public void GetIncrements(int level, double[][][] increments)
        {
            if (currIndexVector == null)
                currIndexVector = new int[Dimension];

            if (prevIndexVector == null)
                prevIndexVector = new int[Dimension];

            int[] dimStepsNum;

            if (level == 0)
                dimStepsNum = dimMultpsRoot;
            else
                dimStepsNum = dimMultps;

            // traversal of all meaningful combinations of currSI and prevSI indexes
            for (int currSI = 1; currSI < increments.Length; currSI++)
            {
                DecomposeRSI(currSI, currIndexVector, dimStepsNum);

                for (int prevSI = 0; prevSI < currSI; prevSI++)
                {
                    DecomposeRSI(prevSI, prevIndexVector, dimStepsNum);

                    // set the Increments components to scaled steps at the given level
                    Buffer.BlockCopy(scaledSteps[level], 0, increments[currSI][prevSI], 0, VectorSize);
                    
                    for (int i = 0; i < Dimension; i++)
                    {
                        prevIndexVector[i] = -prevIndexVector[i];
                        prevIndexVector[i] += currIndexVector[i];

                        increments[currSI][prevSI][i] *= prevIndexVector[i];
                    }
                }
            }
        }

        /// <summary>
        /// Returns array of vector increments between centers of voxels with one direct ancestor;
        /// Vector increments array indexing: [tip_index][origin_index][component_index]
        /// </summary>
        /// <param name="level">The level of voxels which centers are connected by the vector increments</param>
        /// <returns></returns>
        public double[][][] GetIncrements(int level)
        {
            // Increments length is set to tessMultp to provide 
            // compliance with relative SI of a voxel
            var Increments = new double[tessMultp][][];

            for (int i = 1; i < Increments.Length; i++)
            {
                var normInc = new double[i][];
                Increments[i] = normInc;

                for (int n = 0; n < i; n++)
                {
                    normInc[n] = new double[Dimension];
                }
            }

            GetIncrements(level, Increments);

            return Increments;
        }

        /// <summary>
        /// Calculates vector increments between centers of voxels with one direct ancestor;
        /// All vector increments originate from the center of the first child of the ancestor
        /// </summary>
        /// <param name="level">The level of voxels which centers are connected by the vector increments</param>
        /// <param name="increments">Array to storing results; should be initialized</param>
        /// <returns></returns>
        public void GetSerialIncrements(int level, double[][] increments)
        {
            if (currIndexVector == null)
                currIndexVector = new int[Dimension];

            int[] dimStepsNum;

            if (level == 0)
                dimStepsNum = dimMultpsRoot;
            else
                dimStepsNum = dimMultps;
            
            for (int currSI = 1; currSI < increments.Length; currSI++)
            {
                DecomposeRSI(currSI, currIndexVector, dimStepsNum);

                // set the Increments components to scaled steps at the given level
                Buffer.BlockCopy(scaledSteps[level], 0, increments[currSI], 0, VectorSize);

                for (int i = 0; i < Dimension; i++)
                {
                    increments[currSI][i] *= currIndexVector[i];
                }
            }
        }

        /// <summary>
        /// Returns array of vector increments between centers of voxels with one direct ancestor;
        /// All vector increments originate from the center of the first child of the ancestor
        /// </summary>
        /// <param name="level">The level of voxels which centers are connected by the vector increments</param>
        /// <returns></returns>
        public double[][] GetSerialIncrements(int level)
        {
            // Increments length is set to tessMultp to provide 
            // compliance with relative SI of a voxel
            var Increments = new double[tessMultp][];

            for (int i = 1; i < Increments.Length; i++)
            {
                Increments[i] = new double[Dimension];
            }

            GetSerialIncrements(level, Increments);

            return Increments;
        }

        //// The inner loop can be simplified
        //public int[] GetDiagonalPairs(int num = -1, int start = 0)
        //{
        //    int diagsMax = 1 << Dimension - 1;

        //    num = num > 0 && num < diagsMax ? num : diagsMax;

        //    int maxInd = diagsMax - num;

        //    start = start > maxInd ? maxInd : start;

        //    int[] pairs = new int[2 * num];

        //    for (int i = 0; i < pairs.Length; i += 2)
        //    {
        //        int ind1 = start;
        //        int ind2 = ~start;

        //        for (int j = 0, ip = i + 1; j < dimMultps.Length; j++)
        //        {
        //            int mult = dimMultps[j] * (tessFactors[j] - 1);

        //            pairs[i] += (ind1 & 1) * mult;
        //            pairs[ip] += (ind2 & 1) * mult;

        //            ind1 >>= 1;
        //            ind2 >>= 1;
        //        }

        //        ++start;
        //    }

        //    return pairs;
        //}

        /// <summary>
        /// Returns an array of pairs of relative serial indexes of voxels 
        /// arranged diagonally on opposite sides of the center of the ancestor;
        /// The maximum allowed dimensionality is equal to 32
        /// </summary>
        /// <param name="num">The required number of diagonals</param>
        /// <param name="start">Index of the first diagonal</param>
        /// <returns></returns>
        public int[] GetDiagonalPairs(int num = -1, int start = 0)
        {
            int diagsMax = 1 << Dimension - 1;

            num = num > 0 && num < diagsMax ? num : diagsMax;

            int maxInd = diagsMax - num;

            start = start > maxInd ? maxInd : start;

            int[] pairs = new int[2 * num];

            int endex = tessMultp - 1;

            for (int i = 0; i < pairs.Length; i += 2)
            {
                int ind = start;

                for (int j = 0; j < dimMultps.Length; j++)
                {
                    int mult = dimMultps[j] * (tessFactors[j] - 1);

                    pairs[i] += (ind & 1) * mult;

                    ind >>= 1;
                }

                pairs[i + 1] = endex - pairs[i];

                ++start;
            }

            return pairs;
        }

        ////### Debug:Voxel.state
        //public int errNum = 0;
        //public int wellNum = 0;
    }
}
