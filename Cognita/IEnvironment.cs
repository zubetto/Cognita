using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Cognita
{
    namespace ReinforcementLearning
    {
        public class ResponseEventArgs : EventArgs
        {
            public readonly double[] EnviroState;

            public readonly double[] LeversState;
        }

        public interface IEnvironment
        {
            // Agent receives state information of the enviro. and takes action by handling this event
            event EventHandler<ResponseEventArgs> Response;

            // Simulated-time interval between the two consecutive Response events
            double ResponseInterval { get; set; }

            void Run(double[] EnviroState, double[] LeversState);
        }

        public interface IInterpreter<T>
        {
            T GetReward(double[] EnviroState);
            T GetReward(AdaptiveGrid<T>.IVoxel voxel);
        }
    }
}
