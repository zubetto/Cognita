using System;
using System.Threading;

namespace Cognita
{
    public class TasksDispenser
    {
        public readonly int TasksNum;
        public readonly int ThreadsNum;
        public readonly int TasksPerThread;
        public readonly int addNum;

        public void ResetDispenser() => counter = 0;

        private int[][] tasksIndexes = null;

        /// <summary>
        /// Block-dispenser; certain number of consecutive tasks are run by one thread
        /// </summary>
        /// <param name="length"></param>
        /// <returns></returns>
        public bool DispenseNext(out int length)
        {
            if (counter < ThreadsNum)
            {
                if (counter < addNum) length = TpTPlus;
                else length = TasksPerThread;

                counter++;
                return true;
            }

            length = 0;
            return false;
        }

        /// <summary>
        /// Interleaving indexes dispenser; each consecutive task is assigned to different thread
        /// </summary>
        /// <param name="indexes"></param>
        /// <returns></returns>
        public bool DispenseNext(out int[] indexes, bool andSave = true)
        {
            if (counter < ThreadsNum)
            {
                if (andSave)
                {
                    if (tasksIndexes[counter] != null)
                    {
                        indexes = tasksIndexes[counter];
                    }
                    else
                    {
                        if (counter < addNum) indexes = new int[TpTPlus];
                        else indexes = new int[TasksPerThread];

                        for (int i = counter, n = 0; i < TasksNum; i += ThreadsNum, n++)
                        {
                            indexes[n] = i;
                        }

                        tasksIndexes[counter] = indexes;
                    }
                }
                else
                {
                    if (counter < addNum) indexes = new int[TpTPlus];
                    else indexes = new int[TasksPerThread];

                    for (int i = counter, n = 0; i < TasksNum; i += ThreadsNum, n++)
                    {
                        indexes[n] = i;
                    }
                }

                counter++;
                return true;
            }

            indexes = null;
            return false;
        }

        private int TpTPlus;
        private int counter;

        public TasksDispenser(int tasksNum, int threadMultp = 1)
        {
            if (tasksNum <= 0)
                throw new ArgumentOutOfRangeException("tasksNum", "The tasksNum should be positive");

            if (threadMultp <= 0)
                throw new ArgumentOutOfRangeException("threadMultp", "The threadMultp should be positive");

            TasksNum = tasksNum;

            ThreadPool.GetMinThreads(out ThreadsNum, out int cpThreads);

            // ### TEST
            //ThreadsNum = 1;

            ThreadsNum *= threadMultp;

            if (tasksNum >= ThreadsNum)
            {
                TasksPerThread = tasksNum / ThreadsNum;
                addNum = tasksNum % ThreadsNum;
            }
            else
            {
                ThreadsNum = tasksNum;
                TasksPerThread = 1;
                addNum = 0;
            }

            if (addNum > 0) TpTPlus = TasksPerThread + 1;

            tasksIndexes = new int[ThreadsNum][];

            counter = 0;
        }
    }
}
