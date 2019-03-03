#!/usr/bin/env julia
using RobotOS
@rosimport nloptcontrol_planner.msg: Trajectory
rostypegen()
using nloptcontrol_planner.msg


"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/2/2019, Last Modified: 2/2/2019 \n
--------------------------------------------------------------------------------------\n
"""
function loop(pub)
    loop_rate = Rate(10) # run as fast as it can
    while !is_shutdown()

        Z = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
        t = RobotOS.get_param("state/t")

        traj = Trajectory()
        traj.t = [0.0,0.6328947582,1.2657895165,1.8986842747,2.531579033,3.1644737912,3.7973685494,4.4302633077,5.063158065,	5.6960528242,6.3289475824,6.9618423406,7.5947370989,8.2276318571,8.8605266154,9.4934213736,10.1263161318,10.7592108901,11.3921056483,12.0250004065,12.657895164788409]
        traj.x = Z
        traj.y = [-1.1698564724935023e-05,0.7830982331,2.3317959619,4.6074300603,6.8067349463,8.1065251041,8.4590069764,8.6521984658,9.5224302204,	11.3767415567,	14.1737646032,	17.8228290502,	22.2507568046,	27.3969539094,	33.211159302,	39.6514852318,	46.6822504822,	54.2711321968,	62.3882406697,	71.0011561631,	 80.05428345258595]
        traj.v = Z
        traj.r = Z
        traj.psi = Z
        traj.sa = Z
        traj.ux = [1,2,3,4,5,5,5,5,5,5,5,4,3,2,1,1,1,1,1,1,1]
        #traj.ux = [0.8218122588327597,1.6553829204,	3.2416463269	3.9520399693	3.0050030775	1.1145068917	0.0053203494	0.5923533277	2.1246669159	3.6931812751	5.1042675509	6.3883601032	7.5688343612	8.6631787057	9.686064799	10.6471105907	11.554665725	12.4135591675	13.2271553951	13.9821810853	 14.617658064893298]
        traj.ax = Z
        traj.sr = Z
        traj.jx = Z

        publish(pub, traj)

    rossleep(loop_rate)  # sleep for leftover time
    end  # while()
end
"""
--------------------------------------------------------------------------------------\n
Author: Huckleberry Febbo, Graduate Student, University of Michigan
Date Create: 2/2/2019, Last Modified: 2/2/2019 \n
--------------------------------------------------------------------------------------\n
"""
function main()
  println("initializing testTrajectory node ...")
  init_node("testTrajectory")

  # message for solution to optimal control problem
  plannerNamespace = RobotOS.get_param("system/nloptcontrol_planner/namespace")
  pub = Publisher{Trajectory}(string(plannerNamespace,"/control"), queue_size=10)

  loop(pub)
end
if !isinteractive()
    main()
end
