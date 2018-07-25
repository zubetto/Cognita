using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Cognita
{
    namespace SupervisedLearning
    {
        public class PostalServiceZipCode : IPointsSource
        {
            private void ReadConfig(string filename)
            {
                /* Config file structure:
                 * dimension                // int
                 * "points" OR "labels"     // specifies what is located at the first place in each line
                 * positiveDigit            // digit that should be labled with true
                 * negativeDigit            // digit that should be labled with false; set -1 for the "one-vs-all"
                 * 'separator'              // character enclosed in single quotes
                 */

                StreamReader SR = null;

                try
                {
                    SR = new StreamReader(filename);


                }
                finally
                {
                    if (SR != null) SR.Close();
                }
            }

            public PostalServiceZipCode(string configFile, string dataFile)
            {

            }

            //----------------------------------
            #region IPointsSource Implementation

            private const int dimension = 2;
            private int batchLength;

            private double[][] loadedPoints;
            private bool[] loadedLabels;

            private int trainingNum;
            private int testNum;

            private int posNum;
            private int negNum;

            private int loadedLength = 0;
            private DataSet loadedData = DataSet.None;
            
            private int[] indsTraining = new int[2] { -1, 0 };
            private int[] indsTest = new int[2] { -1, 0 };

            #endregion
        }
    }
}
