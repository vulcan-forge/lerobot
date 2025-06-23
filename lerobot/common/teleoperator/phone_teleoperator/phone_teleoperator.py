        # Mapping parameters
        self.quat_RP = None
        self.translation_RP = None
        
        # gRPC server and pose service (to be initialized in connect())
        self.grpc_server = None
        self.pose_service = None 