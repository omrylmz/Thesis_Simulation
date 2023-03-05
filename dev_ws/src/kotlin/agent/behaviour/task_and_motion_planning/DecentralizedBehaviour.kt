package agent.behaviour.task_and_motion_planning

import com.google.gson.GsonBuilder
import java.util.*
import kotlin.math.abs
import kotlin.math.max
import kotlin.collections.ArrayList

import agent.AgentImp
import environment.Perception
import environment.Coordinate
import environment.world.agent.AgentRep
import environment.world.destination.DestinationRep
import environment.world.energystation.EnergyStationRep
import environment.world.packet.PacketRep
import environment.world.pheromone.PheromoneRep
import environment.world.wall.WallRep


data class DecentralizedPDDLPlan(val time: Int, val plan: MutableMap<String, MutableList<String>>)

class DecentralizedBehaviour : LTDBehaviourHttp() {

    var timeStep: Int = 0
    val gson = GsonBuilder().setPrettyPrinting().create()
    lateinit var multiAgentPlan: MutableMap<String, MutableList<String>>
    lateinit var singlePlan: MutableList<String>
    lateinit var simpleSinglePlan: MutableList<String>
    lateinit var pickedPackets: MutableList<Pair<Int, Int>> //Init in the init()
    lateinit var percept: Perception
    lateinit var nextAct: String
    lateinit var nextLoc: Pair<Int, Int>
    lateinit var neighbours: MutableList<Pair<Int, Int>>

    var agents = mapOf<Pair<Int, Int>, AgentRep>()
    var agentLocs = mutableSetOf<Pair<Int, Int>>()
    var agentsData = mutableMapOf<Pair<Int, Int>, String>()
    var agentReps = mutableSetOf<AgentRep>()
    var agentsJson: Map<String, List<String>> = mapOf<String, List<String>>()

    var packets = mutableMapOf<Pair<Int, Int>, PacketRep>()
    var packetLocs = mutableSetOf<Pair<Int, Int>>()
    var packetsData = mutableMapOf<Pair<Int, Int>, String>()
    var packetReps = mutableSetOf<PacketRep>()
    var packetsJson: Map<String, List<String>> = mapOf<String, List<String>>()

    var pheromones = mutableMapOf<Pair<Int, Int>, PheromoneRep>()
    var pheromoneLocs = mutableSetOf<Pair<Int, Int>>()
    var pheromonesData = mutableMapOf<Pair<Int, Int>, String>()
    var pheromoneReps = mutableSetOf<PheromoneRep>()
    var pheromonesJson: Map<String, List<String>> = mapOf<String, List<String>>()

    var walls = mapOf<Pair<Int, Int>, WallRep>()
    var wallLocs = mutableSetOf<Pair<Int, Int>>()
    var wallReps = mutableSetOf<WallRep>()
    var wallsJson: Map<String, List<String>> = mapOf<String, List<String>>()

    var dests = mutableMapOf<Pair<Int, Int>, DestinationRep>()
    var destLocs = mutableSetOf<Pair<Int, Int>>()
    var destsData = mutableMapOf<Pair<Int, Int>, String>()
    var destReps = mutableSetOf<DestinationRep>()
    var destsJson: Map<String, List<String>> = mapOf<String, List<String>>()

    var engStations = mutableMapOf<Pair<Int, Int>, EnergyStationRep>()
    var engStationLocs = mutableSetOf<Pair<Int, Int>>()
    var engStationsData = mutableMapOf<Pair<Int, Int>, String>()
    var engStationReps = mutableSetOf<EnergyStationRep>()
    var engStationsJson: Map<String, List<String>> = mapOf<String, List<String>>()


    override fun communicate(agent: AgentImp) {
        if (timeStep == 0) {
            getState(agent)
            val headerJson = mapOf(
                "time" to timeStep,
                "dimension" to listOf(agent.getEnvironment().getWidth(), agent.getEnvironment().getHeight()),
                "agent_ID" to agent.getID(),
                "agent_pos" to listOf(agent.getX(), agent.getY())
            )
            val bodyList =
                listOf(agentsJson, packetsJson, pheromonesJson, wallsJson, destsJson, engStationsJson)//.filterNotNull()
            val bodyJson = mutableMapOf<String, List<String>>()
            bodyList.filterNot {
                it.any { it2 ->
                    it2.value.isEmpty()
                }
            }.forEach { it ->
                it.forEach { k, v ->
                    bodyJson[k] = v
                }
            }
            val state = mapOf("header" to headerJson, "body" to bodyJson)
            val stateJson = gson.toJson(state)
            println(stateJson)

            val agentColor =
                parseColor(agent.getEnvironment().getAgentWorld().getAgent(agent.getID()).getColor().get().toString())
            val respBody = super.httpPOSTResponse(stateJson, "http://localhost:8888/" + agentColor)
            println(respBody)
            // val respBody = loadPlanJSON(agent)
            loadPlanJSON(agent, respBody)

            pickedPackets = mutableListOf<Pair<Int, Int>>()
        } else {
            val msgs = agent.getMessages().mapNotNull { it.getMessage() }
            msgs.forEach {
                if (it.startsWith("picked")) {
                    val picked = toPair("(" + it.split("(").toList().get(1))
                    pickedPackets.add(picked)
                }
            }
            agent.clearMessages()

            percept = agent.getPerception()
            nextAct = simpleSinglePlan.get(0)
            nextLoc = toPair("(" + nextAct.split("(").toList().get(1))

            ///////////////////////////////////////////////////////////////////////

            if (nextAct.startsWith("wait ")) {
                pickedPackets.forEach {
                    if (it.first == nextLoc.first && it.second == nextLoc.second) {
                        // if (it.contains(nextAct.substring(5))) {
                        simpleSinglePlan.removeFirst()
                        nextAct = simpleSinglePlan.get(0)
                        nextLoc = toPair("(" + nextAct.split("(").toList().get(1))
                    }
                }
            }

            if (nextAct.startsWith("move")) {
                // Sanity check
                if (nextLoc.first == agent.getX() && nextLoc.second == agent.getY()) {
                    simpleSinglePlan.removeFirst()
                    nextAct = simpleSinglePlan.get(0)
                    nextLoc = toPair("(" + nextAct.split("(").toList().get(1))
                }
            }

            if (nextAct.startsWith("wait ")) {
                pickedPackets.forEach {
                    if (it.first == nextLoc.first && it.second == nextLoc.second) {
                        // if (it.contains(nextAct.substring(5))) {
                        simpleSinglePlan.removeFirst()
                        nextAct = simpleSinglePlan.get(0)
                        nextLoc = toPair("(" + nextAct.split("(").toList().get(1))
                    }
                }
            }

            if (nextAct.startsWith("move")) {
                if (nextLoc.first == agent.getX() && nextLoc.second == agent.getY()) {
                    simpleSinglePlan.removeFirst()
                    nextAct = simpleSinglePlan.get(0)
                    nextLoc = toPair("(" + nextAct.split("(").toList().get(1))
                }
            }

            if (nextAct.startsWith("pick ")) {
                agentReps.forEach {
                    if (it.getID() != agent.getID()) {
                        agent.sendMessage(it, "picked " + nextLoc.toString())
                    }
                }
            }
        }
    }


    override fun act(agent: AgentImp) {
        if (timeStep == 0) {
            agent.skip()
        } else {
            if (nextAct.startsWith("wait ")) {
                simpleSinglePlan.add(0, "move (" + agent.getX().toString() + ", " + agent.getY().toString() + ")")
                randomStep(agent)
            } else if (nextAct.startsWith("move")) {
                agent.step(nextLoc.first, nextLoc.second)
            } else if (nextAct.startsWith("pick ")) {
                simpleSinglePlan.removeFirst()
                agent.pickPacket(nextLoc.first, nextLoc.second)
            } else if (nextAct.startsWith("place ")) {
                simpleSinglePlan.removeFirst()
                agent.putPacket(nextLoc.first, nextLoc.second)
            }
        }
        timeStep += 1
    }


    fun findNeighbours(agent: AgentImp, dest: Pair<Int, Int>): MutableList<Pair<Int, Int>> {
        val x = agent.getX()
        val y = agent.getY()
        val maxX = agent.getEnvironment().getWidth()
        val maxY = agent.getEnvironment().getHeight()

        var xRange: MutableList<Int>
        if (x == 0) {
            xRange = mutableListOf(0, 1)
        } else if (x == maxX) {
            xRange = mutableListOf(maxX - 1, maxX)
        } else {
            xRange = mutableListOf(x - 1, x, x + 1)
        }

        var yRange: MutableList<Int>
        if (y == 0) {
            yRange = mutableListOf(0, 1)
        } else if (y == maxY) {
            yRange = mutableListOf(maxY - 1, maxY)
        } else {
            yRange = mutableListOf(y - 1, y, y + 1)
        }

        var neighbours: MutableList<Pair<Int, Int>> = mutableListOf()
        xRange.forEach { it ->
            yRange.forEach { it2 ->
                val cell = percept.getCellPerceptionOnAbsPos(it, it2)
                if (cell != null && cell.isWalkable()) {
                    neighbours.add(Pair<Int, Int>(it, it2))
                }
            }
        }
        neighbours =
            neighbours.sortedBy { max(abs(it.first - dest.first), abs(it.second - dest.second)) }.toMutableList()
        return neighbours
    }


    fun findNeighbourAgents(agent: AgentImp): MutableList<Int> {
        val x = agent.getX()
        val y = agent.getY()
        val maxX = agent.getEnvironment().getWidth()
        val maxY = agent.getEnvironment().getHeight()

        var xRange: MutableList<Int>
        if (x == 0) {
            xRange = mutableListOf(0, 1, 2)
        } else if (x == 1) {
            xRange = mutableListOf(0, 1, 2, 3)
        } else if (x == maxX) {
            xRange = mutableListOf(maxX - 2, maxX - 1, maxX)
        } else if (x == maxX - 1) {
            xRange = mutableListOf(maxX - 3, maxX - 2, maxX - 1, maxX)
        } else {
            xRange = mutableListOf(x - 2, x - 1, x, x + 1, x + 2)
        }

        var yRange: MutableList<Int>
        if (y == 0) {
            yRange = mutableListOf(0, 1, 2)
        } else if (y == 1) {
            yRange = mutableListOf(0, 1, 2, 3)
        } else if (y == maxY) {
            yRange = mutableListOf(maxY - 2, maxY - 1, maxY)
        } else if (y == maxY - 1) {
            yRange = mutableListOf(maxY - 3, maxY - 2, maxY - 1, maxY)
        } else {
            yRange = mutableListOf(y - 2, y - 1, y, y + 1, y + 2)
        }

        var agents: MutableList<Int> = mutableListOf()
        var allAgents = agent.seeAgents().mapKeys { toPair(it.key) }
        xRange.forEach { it ->
            yRange.forEach { it2 ->
                allAgents.forEach { k, v ->
                    if (k.first == it && k.second == it2) {
                        agents.add(v.getID())
                    }
                }
            }
        }
        return agents
    }


    fun findViaPoint(start: Pair<Int, Int>, end: Pair<Int, Int>): MutableList<Pair<Int, Int>> {
        var xRange: MutableList<Int>
        if (start.first - end.first == 2) {
            xRange = mutableListOf(end.first + 1)
        } else if (start.first - end.first == 1) {
            xRange = mutableListOf(end.first, start.first)
        } else if (start.first == end.first) {
            xRange = mutableListOf(start.first)
        } else if (end.first - start.first == 1) {
            xRange = mutableListOf(start.first, end.first)
        } else {
            xRange = mutableListOf(start.first + 1)
        }

        var yRange: MutableList<Int>
        if (start.second - end.second == 2) {
            yRange = mutableListOf(end.second + 1)
        } else if (start.second - end.second == 1) {
            yRange = mutableListOf(end.second, start.second)
        } else if (start.second == end.second) {
            yRange = mutableListOf(start.second)
        } else if (end.second - start.second == 1) {
            yRange = mutableListOf(start.second, end.second)
        } else {
            yRange = mutableListOf(start.second + 1)
        }

        var vias: MutableList<Pair<Int, Int>> = mutableListOf()
        xRange.forEach { it ->
            yRange.forEach { it2 ->
                val cell = percept.getCellPerceptionOnAbsPos(it, it2)
                if (cell != null && cell.isWalkable()) {
                    if (start.first != it || start.second != it2) {
                        if (end.first != it || end.second != it2) {
                            vias.add(Pair<Int, Int>(it, it2))
                        }
                    }
                }
            }
        }
        return vias
    }


    private fun loadPlanJSON(agent: AgentImp, respBody: String) {
        percept = agent.getPerception()
        val (t, p) = gson.fromJson(respBody, CentralizedPDDLPlan::class.java)
        multiAgentPlan = p
        val key = "agent_" + agent.getX().toString() + '-' + agent.getY().toString()

        if (multiAgentPlan[key] != null) {
            singlePlan = multiAgentPlan[key] as MutableList<String>
        } else {
            throw IllegalStateException("Multi-agent plan should have a single agent plan for $key")
        }


        simpleSinglePlan = mutableListOf<String>();
        val rgx = """\((\d*?,\s\d*?)\)""".toRegex()
        val rgxLst = """\[(\(.*?\))\]""".toRegex()
        singlePlan.forEach {
            if (it.startsWith("move ")) {
                val matchLst = rgxLst.findAll(it).toList()
                assert(matchLst.size == 1)
                val steps = matchLst[0].groups[1]?.value?.split("""\)[,]\s+""".toRegex())?.toMutableList()

                steps?.removeFirstOrNull()?.let { it2 ->
                    if (simpleSinglePlan.getOrNull(simpleSinglePlan.lastIndex)?.startsWith("place") ?: false) {
                        val prevAct = simpleSinglePlan.getOrNull(simpleSinglePlan.lastIndex - 1)
                        prevAct?.let { it3 ->
                            val prevLoc = toPair("(" + it3.split("(").toList().get(1))
                            steps?.getOrNull(0)?.let { it4 ->
                                val currLoc = toPair(it4.replace("""[,]\s+""".toRegex(), ", ") + ")")
                                if (abs(prevLoc.first - currLoc.first) == 2 || abs(prevLoc.second - currLoc.second) == 2) {
                                    val viaLoc = findViaPoint(prevLoc, currLoc)[0]
                                    simpleSinglePlan.add("move (" + viaLoc.first.toString() + ", " + viaLoc.second.toString() + ")")
                                }
                            }
                        }
                    }
                }
                steps?.removeLastOrNull()
                steps?.forEach { it2 ->
                    it2.replace("""[,]\s+""".toRegex(), ", ")
                    simpleSinglePlan.add("move " + it2 + ")")
                }
            } else if (it.startsWith("move-with-carry ")) {
                val matchLst = rgxLst.findAll(it).toList()
                assert(matchLst.size == 1)
                val steps = matchLst[0].groups[1]?.value?.split("""\)[,]\s+""".toRegex())?.toMutableList()

                steps?.removeLastOrNull()
                steps?.forEach { it2 ->
                    it2.replace("""[,]\s+""".toRegex(), ", ")
                    simpleSinglePlan.add("move-with-carry " + it2 + ")")
                }
            } else if (it.startsWith("move-conditional ")) {
                val match = rgx.findAll(it).toList()
                val xy = match.last().groups[1]?.value?.trim()?.split(""",\s+""".toRegex())?.toMutableList()
                simpleSinglePlan.add("wait " + "(" + xy?.get(0) + ", " + xy?.get(1) + ")")

                val matchLst = rgxLst.findAll(it).toList()
                assert(matchLst.size == 1)
                val steps = matchLst[0].groups[1]?.value?.split("""\)[,]\s+""".toRegex())?.toMutableList()

                steps?.removeFirstOrNull()?.let { it2 ->
                    if (simpleSinglePlan.getOrNull(simpleSinglePlan.lastIndex)?.startsWith("place") ?: false) {
                        val prevAct = simpleSinglePlan.getOrNull(simpleSinglePlan.lastIndex - 1)
                        prevAct?.let { it3 ->
                            val prevLoc = toPair("(" + it3.split("(").toList().get(1))
                            steps?.getOrNull(0)?.let { it4 ->
                                val currLoc = toPair(it4.replace("""[,]\s+""".toRegex(), ", ") + ")")
                                if (abs(prevLoc.first - currLoc.first) == 2 || abs(prevLoc.second - currLoc.second) == 2) {
                                    val viaLoc = findViaPoint(prevLoc, currLoc)[0]
                                    simpleSinglePlan.add("move (" + viaLoc.first.toString() + ", " + viaLoc.second.toString() + ")")
                                }
                            }
                        }
                    }
                }
                steps?.removeLastOrNull()
                steps?.forEach { it2 ->
                    it2.replace("""[,]\s+""".toRegex(), ", ")
                    simpleSinglePlan.add("move-conditional " + it2 + ")")
                }
            } else if (it.startsWith("move-with-carry-conditional ")) {
                val match = rgx.findAll(it).toList()
                assert(match.size == 2)
                val xy = match.last().groups[1]?.value?.trim()?.split(""",\s+""".toRegex())?.toMutableList()
                simpleSinglePlan.add("wait " + "(" + xy?.get(0) + ", " + xy?.get(1) + ")")

                val matchLst = rgxLst.findAll(it).toList()
                assert(matchLst.size == 1)
                val steps = matchLst[0].groups[1]?.value?.split("""\)[,]\s+""".toRegex())?.toMutableList()

                steps?.removeLastOrNull()
                steps?.forEach { it2 ->
                    it2.replace("""[,]\s+""".toRegex(), ", ")
                    simpleSinglePlan.add("move-with-carry-conditional " + it2 + ")")
                }
            } else if (it.startsWith("pick ")) {
                val match = rgx.findAll(it).toList()
                assert(match.size == 1)
                val location = "(" + match[0].groups[1]?.value?.trim()?.replace(""",\s+""".toRegex(), ", ") + ")"
                simpleSinglePlan.add("pick " + location)
            } else if (it.startsWith("place ")) {
                val match = rgx.findAll(it).toList()
                assert(match.size == 1)
                val location = "(" + match[0].groups[1]?.value?.trim()?.replace(""",\s+""".toRegex(), ", ") + ")"
                simpleSinglePlan.add("place " + location)
            }
        }
        simpleSinglePlan.add("wait (-1, -1)")
    }


    fun getState(agent: AgentImp) {
        getStatic(agent)
        getDynamic(agent)
    }


    fun getStatic(agent: AgentImp) {
        // Create walls' state JSON
        walls = agent.seeWalls().mapKeys { toPair(it.key) }
        walls.keys.forEach { if (it !in wallLocs) wallLocs.add(it) }
        walls.values.forEach { if (it !in wallReps) wallReps.add(it) }
        wallsJson = toWorldJSONField("WallRep", wallLocs)

        // Create destinations' state JSON
        agent.seeDestinations().mapKeys { toPair(it.key) }.forEach { (k, v) ->
            if (k !in dests) {
                dests.put(k, v); destsData.put(k, v.getColor().toString())
            }
        }
        // var temp = dests.keys.toMutableSet().map{it to dests[it]!.getColor.getString()}.toMap()
        dests.keys.forEach { if (it !in destLocs) destLocs.add(it) }
        dests.values.forEach { if (it !in destReps) destReps.add(it) }
        destsJson = toWorldJSONField("DestinationRep", destsData)

        // Create energy stations' state JSON
        agent.seeEnergyStations().mapKeys { toPair(it.key) }
            .forEach { (k, v) -> if (k !in engStations) engStations.put(k, v) }
        engStations.keys.forEach { if (it !in engStationLocs) engStationLocs.add(it) }
        engStations.values.forEach { if (it !in engStationReps) engStationReps.add(it) }
        engStationsJson = toWorldJSONField("EnergyStationRep", engStationLocs)
    }

    fun getDynamic(agent: AgentImp) {
        // Create agents' state JSON
        // agents = agent.getEnvironment().getAgentWorld().getAgents().mapKeys {toPair(it.key)}
        agents = agent.seeAgents().mapKeys { toPair(it.key) }
        agents.forEach { (k, v) ->
            agentsData.put(
                k,
                agent.getEnvironment().getAgentWorld().getAgent(v.getID()).getColor().get().toString()
            )
        }
        agentsData.put(
            Pair<Int, Int>(agent.getX(), agent.getY()),
            agent.getEnvironment().getAgentWorld().getAgent(agent.getID()).getColor().get().toString()
        )
        agents.keys.forEach { if (it !in agentLocs) agentLocs.add(it) }
        agents.values.forEach { if (it !in agentReps) agentReps.add(it) }
        agentsJson = toWorldJSONField("AgentRep", agentsData)

        // Create agents' state JSON
        agent.seePackets().mapKeys { toPair(it.key) }.forEach { (k, v) ->
            if (k !in packets) {
                packets.put(k, v); packetsData.put(k, v.getColor().toString())
            }
        }
        packets.keys.forEach { if (it !in packetLocs) packetLocs.add(it) }
        packets.values.forEach { if (it !in packetReps) packetReps.add(it) }
        packetsJson = toWorldJSONField("PacketRep", packetsData)

        // Create pheromones' state JSON
        agent.seePheromones().mapKeys { toPair(it.key) }
            .forEach { (k, v) -> if (k !in pheromones) pheromones.put(k, v) }
        pheromones.keys.forEach { if (it !in pheromoneLocs) pheromoneLocs.add(it) }
        pheromones.values.forEach { if (it !in pheromoneReps) pheromoneReps.add(it) }
        pheromonesJson = toWorldJSONField("PheromoneRep", pheromoneLocs)
    }


    private fun toPair(str: String): Pair<Int, Int> {
        var trimmed = str.replace("(", "")
        trimmed = trimmed.replace(")", "")
        var pair = trimmed.split(",".toRegex()).map { it.trim() }.map { it.toInt() }
        return Pair<Int, Int>(pair[0], pair[1])
    }


    private fun toWorldJSONField(key: String, set: Set<Pair<Int, Int>>): Map<String, List<String>> {
        val setStr = set.map { it.toString() }
        return mapOf(
            key to if (set.isEmpty()) {
                emptyList<String>()
            } else {
                setStr.toList()
            }
        )
    }


    private fun toWorldJSONField(key: String, map: Map<Pair<Int, Int>, String>): Map<String, List<String>> {
        val setStr = map.mapKeys { it.key.toString() }.map { (k, v) -> k + ": " + parseColor(v) }.toSet()
        return mapOf(
            key to if (map.isEmpty()) {
                emptyList<String>()
            } else {
                setStr.toList()
            }
        )
    }


    private fun parseColor(color: String): String {
        when (color) {
            "java.awt.Color[r=0,g=0,b=0]" -> return "BLACK"
            "java.awt.Color[r=255,g=0,b=0]" -> return "RED"
            "java.awt.Color[r=0,g=255,b=0]" -> return "GREEN"
            "java.awt.Color[r=0,g=0,b=255]" -> return "BLUE"
            "java.awt.Color[r=255,g=255,b=0]" -> return "YELLOW"
            "java.awt.Color[r=255,g=0,b=255]" -> return "MAGENTA"
            "java.awt.Color[r=255,g=175,b=175]" -> return "PINK"
        }
        return ""
    }


    fun randomStep(agent: AgentImp) {
        // Potential moves an agent can make (radius of 1 around the agent)
        val moves: List<Coordinate?> = ArrayList(
            java.util.List.of(
                Coordinate(1, 1), Coordinate(-1, -1),
                Coordinate(1, 0), Coordinate(-1, 0),
                Coordinate(0, 1), Coordinate(0, -1),
                Coordinate(1, -1), Coordinate(-1, 1)
            )
        )

        // Shuffle moves randomly
        Collections.shuffle(moves)

        // Check for viable moves
        for (move in moves) {
            val perception = agent.perception
            val x = move!!.x
            val y = move.y

            // If the area is null, it is outside of the bounds of the environment
            //  (when the agent is at any edge for example some moves are not possible)
            if (perception.getCellPerceptionOnRelPos(x, y) != null && perception.getCellPerceptionOnRelPos(
                    x,
                    y
                ).isWalkable
            ) {
                agent.step(agent.x + x, agent.y + y)
                return
            }
        }

        // No viable moves, skip turn
        agent.skip()
    }
}