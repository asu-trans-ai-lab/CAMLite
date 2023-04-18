# @author       Jiawei Lu (jiaweil9@asu.edu)
#               Xuesong Zhou (xzhou74@asu.edu)
# @time         2020/12/21 13:52
# @desc         [script description]

from collections import deque
import networkx as nx

class MesoNode:
    def __init__(self):
        self.name = ''
        self.node_id = 0
        self.node_seq_no = 0
        self.zone_id = None
        self.control_type = 0
        self.control_type_name = ''
        self.cycle_length_in_second = 0
        self.x = 0.0
        self.y = 0.0
        self.m_outgoing_link_list = []
        self.m_incoming_link_list = []
        self.outgoing_link_list = []
        self.incoming_link_list = []


class MesoLink:
    def __init__(self):
        self.name = ''
        self.link_id = 0
        self.link_seq_no = 0
        self.from_node_id = 0
        self.to_node_id = 0
        self.from_node = None
        self.to_node = None
        self.length = 0.0
        self.number_of_lanes = 0
        self.speed_limit = 0.0
        self.lane_cap = 0.0
        self.base_volume = 0.0      # fixed route vehicles
        self.flow_volume = 0.0
        self.BPR_alpha = 0.15
        self.BPR_beta = 4.0
        self.link_capacity = 0.0
        self.travel_time = 0.0
        self.cost = 0.0

        self.link_key = 0
        self.isconnector = False
        self.geometry_list = []  # vector<float*> geometry_vector;

        self.micro_node_list = []  # vector<vector<int>> micro_node_set;			//micro node id, lane by lane;
        self.micro_link_list = []  # vector<int> micro_link_set;			//micro link id;
        self.control_micro_link_list = []       # the first micro link in each lane

        self.free_flow_travel_time_in_min = 0.0
        self.free_flow_travel_time_in_sim_interval = 0
        self.m_active_agent_queue = deque()

        self.graph = nx.DiGraph()
        self.micro_incoming_node_id_list = None
        self.micro_outgoing_node_id_list = None
        self.turning_node_seq_no_dict = {}  # std::map<int, vector<int>> turning_node_seq_no_dict;             //meso_link_seq_no:micro_node_seq_no
        self.estimated_cost_tree_for_each_movement = {}  # std::map<int, map<int,float>> estimated_cost_tree_for_each_movement;     //meso_link_seq_no : node_seq_no: cost

        self.mean_speed_list = [4.0, 12.1, 20.1, 28.2, 36.2, 44.3, 52.3, 60.3, 68.4, 76.4, 84.5, 92.5, 100.6, 108.6, 116.7, 124.7]
        self.mean_emission_list = [7668.5, 13970.2, 21033.6, 21033.6, 21033.6, 53669.6, 53669.6, 53669.6, 53669.6, 53669.6, 53979.1, 
                                   53979.1, 53979.1, 53979.1, 53979.1, 53979.1]

        self.current_speed_idx = 0
        self.td_speed_time_list = []
        self.td_speed_value_list = []

        self.travel_cell_length = None
        self.change_cell_length = None
        self.accessible_nodes_dict = {}
        self.lane_nodes_dict = {}

    @property
    def travel_change_cell_length_difference(self):
        return None if self.travel_cell_length is None or self.change_cell_length is None else self.change_cell_length - self.travel_cell_length

    def CalculateBPRFunctionAndCost(self):
        self.travel_time = self.free_flow_travel_time_in_min * (1 + self.BPR_alpha * pow(self.flow_volume / self.link_capacity, self.BPR_beta))
        speed = self.length / 1000 / self.travel_time * 60
        emission_rate = self.mean_emission_list[min(range(len(self.mean_speed_list)), key=lambda i: abs(self.mean_speed_list[i]-speed))]
        emission = emission_rate * self.travel_time / 60
        self.cost = emission

        # self.cost = self.travel_time

    def setLinkKey(self):
        self.link_key = self.from_node_id * 100000 + self.to_node_id

    def initializeMicroNodeList(self):
        for i in range(self.number_of_lanes): self.micro_node_list.append([])

    def Initialization(self):
        self.link_capacity = self.number_of_lanes * self.lane_cap
        self.free_flow_travel_time_in_min = self.length / self.speed_limit * 0.06


class Zone:
    def __init__(self, zone_id):
        self.zone_id = zone_id
        self.meso_node_list = []
        self.outgoing_meso_node_list = []
        self.incoming_meso_node_list = []


class MicroNode:
    def __init__(self):
        self.node_id = 0
        self.node_seq_no = 0
        self.x = 0.0
        self.y = 0.0
        self.meso_link_id = 0
        self.lane_no = 0
        self.m_outgoing_link_keep = None
        self.m_outgoing_link_change_list = []
        self.m_outgoing_link_list = []
        self.m_incoming_link_list = []
        self.outgoing_link_list = []
        self.incoming_link_list = []
        self.available_sim_interval = 0

        self.outgoing_mesolinks = set()

    @property
    def downstream_mesolinks(self):
        if self.m_outgoing_link_keep is None:
            return set()
        elif self.outgoing_mesolinks:
            return self.outgoing_mesolinks
        else:
            return self.m_outgoing_link_keep.to_node.downstream_mesolinks


class MicroLink:
    def __init__(self):
        self.link_id = 0
        self.link_seq_no = 0
        self.from_node_id = 0
        self.to_node_id = 0
        self.from_node = None
        self.to_node = None
        self.meso_link = None
        self.lane_no = 0
        self.length = 0.0
        self.speed_limit = None
        self.free_flow_travel_time_in_min = 0.0
        self.free_flow_travel_time_in_simu_interval = 0
        self.additional_cost = 0.0
        self.cost = 0.0
        self.cell_type = 1  # //1:traveling; 2:changing

    def Initialization(self,number_of_seconds_per_interval):
        self.additional_cost = 0 if self.cell_type == 1 else 2
        self.free_flow_travel_time_in_min = self.length / self.speed_limit * 0.06
        self.free_flow_travel_time_in_simu_interval = max(round(self.free_flow_travel_time_in_min * 60 / number_of_seconds_per_interval), 1)
        self.cost = self.free_flow_travel_time_in_simu_interval + self.additional_cost

    def td_travel_speed(self, sim_time_in_minute):
        if self.meso_link.td_speed_time_list:
            while self.meso_link.current_speed_idx < len(self.meso_link.td_speed_time_list) - 1 and sim_time_in_minute >= self.meso_link.td_speed_time_list[self.meso_link.current_speed_idx+1]:
                self.meso_link.current_speed_idx += 1
            return self.meso_link.td_speed_value_list[self.meso_link.current_speed_idx]
        elif self.speed_limit is not None:
            return self.speed_limit
        else:
            return self.meso_link.speed_limit


class Network:
    def __init__(self):
        self.number_of_micro_nodes = 0
        self.number_of_micro_links = 0
        self.number_of_meso_nodes = 0
        self.number_of_meso_links = 0

        self.micro_node_list = []
        self.micro_link_list = []
        self.meso_node_list = []
        self.meso_link_list = []

        self.meso_node_id_to_seq_no_dict = {}
        self.micro_node_id_to_seq_no_dict = {}
        self.meso_link_id_to_seq_no_dict = {}
        self.micro_link_id_to_seq_no_dict = {}
        self.meso_link_key_to_seq_no_dict = {}

        self.zone_dict = {}


    def initialization(self, args_):
        print('  initializing network')

        # ------------------Zone------------------------#
        for _, zone in self.zone_dict.items():
            for mesonode in zone.meso_node_list:
                if mesonode.m_outgoing_link_list:
                    zone.outgoing_meso_node_list.append(mesonode)
                if mesonode.m_incoming_link_list:
                    zone.incoming_meso_node_list.append(mesonode)


        # ----------------MesoLink----------------------#
        # incoming and outgoing nodes identification
        for mesolink in self.meso_link_list:
            potential_micro_incoming_node_id_set = set()
            potential_micro_outgoing_node_id_set = set()

            for micro_link_id in mesolink.micro_link_list:
                micro_link = self.micro_link_list[self.micro_link_id_to_seq_no_dict[micro_link_id]]
                potential_micro_incoming_node_id_set.add(micro_link.from_node_id)
                potential_micro_outgoing_node_id_set.add(micro_link.to_node_id)

            mesolink.micro_incoming_node_id_list = list(potential_micro_incoming_node_id_set - potential_micro_outgoing_node_id_set)
            mesolink.micro_outgoing_node_id_list = list(potential_micro_outgoing_node_id_set - potential_micro_incoming_node_id_set)

            # for micro_node_id in mesolink.micro_outgoing_node_id_list:
            #     micronode = self.micro_node_list[self.micro_node_id_to_seq_no_dict[micro_node_id]]
            #     for outgoing_micro_link_id in micronode.m_outgoing_link_list:
            #         outgoing_microlink = self.micro_link_list[self.micro_link_id_to_seq_no_dict[outgoing_micro_link_id]]
            #         micronode.outgoing_mesolinks.add(outgoing_microlink.meso_link)

        for mesolink in self.meso_link_list:
            for downstream_mesolink in mesolink.to_node.outgoing_link_list:
                mesolink.accessible_nodes_dict[downstream_mesolink] = set()
                accessible_nodes_temp = set([self.micro_node_list[self.micro_node_id_to_seq_no_dict[node_id]] for node_id in mesolink.micro_outgoing_node_id_list if node_id in downstream_mesolink.micro_incoming_node_id_list])
                while accessible_nodes_temp:
                    node = accessible_nodes_temp.pop()
                    mesolink.accessible_nodes_dict[downstream_mesolink].add(node)
                    for incoming_microlink in node.incoming_link_list:
                        if incoming_microlink.meso_link is mesolink:
                            accessible_nodes_temp.add(incoming_microlink.from_node)

                mesolink.lane_nodes_dict[downstream_mesolink] = set()
                lane_nodes_temp = set([self.micro_node_list[self.micro_node_id_to_seq_no_dict[node_id]] for node_id in mesolink.micro_outgoing_node_id_list if node_id in downstream_mesolink.micro_incoming_node_id_list])
                while lane_nodes_temp:
                    node = lane_nodes_temp.pop()
                    mesolink.lane_nodes_dict[downstream_mesolink].add(node)
                    for incoming_microlink in node.incoming_link_list:
                        if incoming_microlink.cell_type == 1 and incoming_microlink.meso_link is mesolink:
                            lane_nodes_temp.add(incoming_microlink.from_node)

        # control link identification
        for mesolink in self.meso_link_list:
            for incoming_node_id in mesolink.micro_incoming_node_id_list:
                incoming_node = self.micro_node_list[self.micro_node_id_to_seq_no_dict[incoming_node_id]]
                for microlink_id in incoming_node.m_outgoing_link_list:
                    microlink = self.micro_link_list[self.micro_link_id_to_seq_no_dict[microlink_id]]
                    if microlink.meso_link is mesolink:
                        mesolink.control_micro_link_list.append(microlink)

        for mesolink in self.meso_link_list: mesolink.Initialization()


        # ----------------MicroLink----------------------#
        number_of_seconds_per_interval = args_['sim_step']
        for microlink in self.micro_link_list: microlink.Initialization(number_of_seconds_per_interval)
