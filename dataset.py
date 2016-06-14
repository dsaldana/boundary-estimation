import numpy as np

#### Initial conditions ####
theta0 = 0.1
ti, tf = 0, 100
n = 200
time = np.linspace(ti, tf, n)

atheta = np.array([3., 3.07537688, 3.15075377, 3.22613065,
                   3.30150754, 3.37688442, 3.45226131, 3.52763819,
                   3.60301508, 3.67839196, 3.75376884, 3.82914573,
                   3.90452261, 3.9798995, 4.05527638, 4.13065327,
                   4.20603015, 4.28140704, 4.35678392, 4.4321608,
                   4.50753769, 4.58291457, 4.65829146, 4.73366834,
                   4.80904523, 4.88442211, 4.95979899, 5.03517588,
                   5.11055276, 5.18592965, 5.26130653, 5.33668342,
                   5.4120603, 5.48743719, 5.56281407, 5.63819095,
                   5.71356784, 5.78894472, 5.86432161, 5.93969849,
                   6.01507538, 6.09045226, 6.16582915, 6.24120603,
                   6.31658291, 6.3919598, 6.46733668, 6.54271357,
                   6.61809045, 6.69346734, 6.76884422, 6.84422111,
                   6.91959799, 6.99497487, 7.07035176, 7.14572864,
                   7.22110553, 7.29648241, 7.3718593, 7.44723618,
                   7.52261307, 7.59798995, 7.67336683, 7.74874372,
                   7.8241206, 7.89949749, 7.97487437, 8.05025126,
                   8.12562814, 8.20100503, 8.27638191, 8.35175879,
                   8.42713568, 8.50251256, 8.57788945, 8.65326633,
                   8.72864322, 8.8040201, 8.87939698, 8.95477387,
                   9.03015075, 9.10552764, 9.18090452, 9.25628141,
                   9.33165829, 9.40703518, 9.48241206, 9.55778894,
                   9.63316583, 9.70854271, 9.7839196, 9.85929648,
                   9.93467337, 10.01005025, 10.08542714, 10.16080402,
                   10.2361809, 10.31155779, 10.38693467, 10.46231156,
                   10.53768844, 10.61306533, 10.68844221, 10.7638191,
                   10.83919598, 10.91457286, 10.98994975, 11.06532663,
                   11.14070352, 11.2160804, 11.29145729, 11.36683417,
                   11.44221106, 11.51758794, 11.59296482, 11.66834171,
                   11.74371859, 11.81909548, 11.89447236, 11.96984925,
                   12.04522613, 12.12060302, 12.1959799, 12.27135678,
                   12.34673367, 12.42211055, 12.49748744, 12.57286432,
                   12.64824121, 12.72361809, 12.79899497, 12.87437186,
                   12.94974874, 13.02512563, 13.10050251, 13.1758794,
                   13.25125628, 13.32663317, 13.40201005, 13.47738693,
                   13.55276382, 13.6281407, 13.70351759, 13.77889447,
                   13.85427136, 13.92964824, 14.00502513, 14.08040201,
                   14.15577889, 14.23115578, 14.30653266, 14.38190955,
                   14.45728643, 14.53266332, 14.6080402, 14.68341709,
                   14.75879397, 14.83417085, 14.90954774, 14.98492462,
                   15.06030151, 15.13567839, 15.21105528, 15.28643216,
                   15.36180905, 15.43718593, 15.51256281, 15.5879397,
                   15.66331658, 15.73869347, 15.81407035, 15.88944724,
                   15.96482412, 16.04020101, 16.11557789, 16.19095477,
                   16.26633166, 16.34170854, 16.41708543, 16.49246231,
                   16.5678392, 16.64321608, 16.71859296, 16.79396985,
                   16.86934673, 16.94472362, 17.0201005, 17.09547739,
                   17.17085427, 17.24623116, 17.32160804, 17.39698492,
                   17.47236181, 17.54773869, 17.62311558, 17.69849246,
                   17.77386935, 17.84924623, 17.92462312, 18.])

xx = [-1.97998499320089, -1.99085697323102, -1.99145902337447, -1.98052293185527, -1.95725813232876, -1.92212109326372,
      -1.87659847840394, -1.82210000492045, -1.75877133758197, -1.68517407458760, -1.59920356922782, -1.49971596468081,
      -1.38776191992363, -1.26651707674908, -1.13986011703962, -1.01048966450092, -0.878819742149493,
      -0.743378817295690, -0.602400410284576, -0.455462852178281, -0.304015636955632, -0.150454975472972,
      0.00349996515840277, 0.157825102774127, 0.313719935854393, 0.471894918486995, 0.630895770788895,
      0.786873369191286, 0.935253402671283, 1.07333299762806, 1.20195982814023, 1.32485647065610, 1.44569708317463,
      1.56473254492900, 1.67738063763056, 1.77616308638732, 1.85520499339383, 1.91459505344266, 1.96159801780211,
      2.00741217885815, 2.06094437158069, 2.12318599999055, 2.18575724120467, 2.23486977963300, 2.25866440289998,
      2.25366005325729, 2.22638901522481, 2.18910727198208, 2.15208718099393, 2.11716663589664, 2.07653545340739,
      2.01755411160250, 1.93068075134010, 1.81566012080441, 1.68222068996620, 1.54494531647008, 1.41555870449987,
      1.29735071145661, 1.18493076743138, 1.06902645376479, 0.942890158019954, 0.806095771998182, 0.663532480698365,
      0.520801496777545, 0.379658600500094, 0.236962486519237, 0.0879361956935901, -0.0687050473035959,
      -0.227742716305534, -0.379198955180767, -0.514089025079121, -0.630343147023687, -0.734513363239114,
      -0.837472693276139, -0.946456088344191, -1.05869041989796, -1.16136803214563, -1.23886357322286,
      -1.28321455636646, -1.30120450326770, -1.31289863384510, -1.34153451697158, -1.40022449330757, -1.48340956804598,
      -1.56861495637755, -1.62803307979342, -1.64342692909393, -1.61559780337129, -1.56273903639505, -1.50863758261968,
      -1.46784272722835, -1.43677282512185, -1.39609140017814, -1.32278550070789, -1.20449497250708, -1.04740377424109,
      -0.873119735419649, -0.706787269888070, -0.563896617968852, -0.443664597367272, -0.332371910301449,
      -0.213673359172739, -0.0787831307475297, 0.0698744439595678, 0.223129340830647, 0.373625843276400,
      0.521879832581850, 0.675351916145106, 0.841312311198367, 1.01895762925161, 1.19701734320615, 1.35931118396417,
      1.49493002866393, 1.60596733586000, 1.70682753805812, 1.81462840054216, 1.93654215349857, 2.06271816085144,
      2.17047121839973, 2.23826561652195, 2.26119685825490, 2.25778781622698, 2.26260881149226, 2.30789606522477,
      2.40461302443480, 2.53450637366787, 2.65860592286707, 2.73781732320156, 2.75376083534386, 2.71757245709766,
      2.66146088066389, 2.61816935896691, 2.60077565199915, 2.59496911245610, 2.56825007366367, 2.49017563550573,
      2.35120706613993, 2.16888320572363, 1.97806807391967, 1.81176340794354, 1.68434980992398, 1.58694256273439,
      1.49649426334360, 1.39168684294844, 1.26501801461676, 1.12389360164715, 0.981417773286717, 0.844503285370056,
      0.708279792553672, 0.560810187890086, 0.394409082576719, 0.214836198446786, 0.0408943006865574,
      -0.106262938185506, -0.217165711654996, -0.301370945378322, -0.382003844991626, -0.480000870777498,
      -0.598136109391688, -0.716105856209226, -0.801443651192229, -0.830687250382971, -0.807717848751530,
      -0.766763841677708, -0.756403999287858, -0.812793338162016, -0.937769670968696, -1.09547217163873,
      -1.23043816229323, -1.29719097503438, -1.28410611355331, -1.21749019709510, -1.14358762895597, -1.09949344774035,
      -1.09054764388269, -1.08765926253095, -1.04574549171052, -0.931684323652874, -0.744729316356699,
      -0.517418014724943, -0.297219377908793, -0.120754539663167, 0.00375810857974734, 0.0961888301408781,
      0.188291723707275, 0.303222363054659, 0.444007738847939, 0.597181161343733, 0.747178314439943, 0.890095547409941,
      1.03660828100879, 1.20215681918397, 1.39201104409368, 1.59282849173979, 1.77793163354134, 1.92368612932131,
      2.02589553211353, 2.10413909168297, 2.18959165779861, 2.30325839090941]
yy = [0.282240016119734, 0.132352359120729, -0.0183365701224300, -0.169106931915703, -0.318932384665967,
      -0.466817777549886, -0.612190301818987, -0.754870764862707, -0.894487911267903, -1.02973898847707,
      -1.15818620545068, -1.27700740296148, -1.38441207898951, -1.48079576793494, -1.56867100663913, -1.65112626005527,
      -1.72961149657754, -1.80249642331903, -1.86554874865964, -1.91430711101182, -1.94699211005201, -1.96603034372880,
      -1.97695459331581, -1.98506129111714, -1.99175077237052, -1.99289550166247, -1.98049261763724, -1.94685722187951,
      -1.88895000287185, -1.81021330290794, -1.71873295723832, -1.62278992220013, -1.52649678311479, -1.42816447740829,
      -1.32231574016783, -1.20395072702696, -1.07227056322279, -0.931487152679667, -0.788308657573489,
      -0.647885668718138, -0.510970642108502, -0.374143446792386, -0.232813229528591, -0.0847971359242395,
      0.0680749034921457, 0.221417431962912, 0.371635407680049, 0.518534138565109, 0.665014860663424, 0.813794354428156,
      0.963427715544964, 1.10677215791273, 1.23363870280001, 1.33652067302338, 1.41586237803706, 1.48110918809029,
      1.54615777467618, 1.62143575681486, 1.70734572790829, 1.79341380050426, 1.86418782957349, 1.90866638126693,
      1.92752202074988, 1.93337273954988, 1.94344613067889, 1.96873705683356, 2.00618833128641, 2.03887700993479,
      2.04447655942603, 2.00722998991323, 1.92644112616134, 1.81650937764460, 1.69859901995975, 1.58902819657806,
      1.49129754118879, 1.39624810762515, 1.28972066936081, 1.16253312022173, 1.01644187380821, 0.862631057080058,
      0.714215410985923, 0.578003010748277, 0.450943616892081, 0.323292202883918, 0.185913380551212, 0.0364682440818139,
      -0.119757258176889, -0.274664478698913, -0.423962037877191, -0.570400879660431, -0.720851587377774,
      -0.879035428975101, -1.03925871165876, -1.18636774103407, -1.30316192914665, -1.38119918153736, -1.42793146729908,
      -1.46472005193280, -1.51580598961357, -1.59433998196160, -1.69410164909798, -1.79260869661275, -1.86445535085868,
      -1.89712691505619, -1.89948676797113, -1.89719337334517, -1.91720079306757, -1.97040168538942, -2.04300251802431,
      -2.10223972251368, -2.11347764687202, -2.05881037978232, -1.94631274502222, -1.80469105507670, -1.66682617295574,
      -1.55219353902865, -1.45843629986123, -1.36641120927796, -1.25466743040313, -1.11381628360666, -0.951933149697766,
      -0.788324781912955, -0.640538722255596, -0.513500372671534, -0.397698361152855, -0.276936202869735,
      -0.139739835588464, 0.0136345506174140, 0.173178647766094, 0.328105917968009, 0.475768011519401,
      0.623486285902044, 0.781445194629268, 0.951559602162752, 1.12078722076586, 1.26496551625354, 1.36220759163983,
      1.40782731034945, 1.42062941564341, 1.43508527568928, 1.48278475464097, 1.57396486712698, 1.69082006934288,
      1.79764057743705, 1.86245235047878, 1.87723005599701, 1.86394993085829, 1.86199723550593, 1.90386831805002,
      1.99363896376895, 2.10147083032417, 2.17801603017318, 2.18062373768392, 2.09609213290528, 1.94677519354873,
      1.77687623152844, 1.62754788441274, 1.51573879478505, 1.42882917766446, 1.33701345413831, 1.21456769432394,
      1.05631054758656, 0.879486724164772, 0.711115582551157, 0.570057879429807, 0.455794394940306, 0.350712949149309,
      0.233327265626785, 0.0926882530529106, -0.0659812143071420, -0.227752372580754, -0.381096691989108,
      -0.527403541189061, -0.679303149901692, -0.848448605791254, -1.03156186151027, -1.20573500597445,
      -1.33828904394029, -1.40644067092392, -1.41436983680063, -1.39554022609685, -1.39675533471884, -1.45223429901741,
      -1.56329447526336, -1.69687677783836, -1.80505056830812, -1.85446380323028, -1.84782470548331, -1.82361714458097,
      -1.83323138313956, -1.90868822172973, -2.04040544702560, -2.17888154409281, -2.25996052197330, -2.23913925890710,
      -2.11491770130028, -1.92783231770275, -1.73642964785118, -1.58500759824467, -1.48221363870042]


D = (time, atheta, xx, yy)