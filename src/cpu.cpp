#include "cpu.h"
#include <fstream>

CPU::CPU(uint32_t numArchRegs, uint32_t numPhysicalRegs,
		uint32_t robEntries, uint32_t width, uint32_t numLSQEntries) :
	numArchRegs(numArchRegs), numPhysicalRegs(numPhysicalRegs),
	robEntries(robEntries), width(width), numLSQEntries(numLSQEntries),
	archMappingTable("archMapTable", numArchRegs, numPhysicalRegs),
	mapTable("Mapping Table", numArchRegs, numPhysicalRegs),
	rob(robEntries), freeList(numArchRegs, numPhysicalRegs),
	fetchStage("fetch", width),
	decodeStage("decode", width),
	dispatchStage("dispatch", width),
	issueStage("issue", width),
	executeStage("execute", width),
	completeStage("complete", width),
	retireStage("retire", width),
	fetchPtr(0), isFetching(true),
	hasProgress(false), cycle(0)
{
	reservationStations.push_back(new ReservationStation("ALU", RSType_ALU, 1));
	reservationStations.push_back(new ReservationStation("ALU", RSType_ALU, 1));
	reservationStations.push_back(new ReservationStation("LOAD", RSType_LOAD, 2));
	reservationStations.push_back(new ReservationStation("STORE", RSType_STORE, 2));
}

CPU::~CPU() {
}

void CPU::addInstruction(char type, uint32_t srcOp1,
		uint32_t srcOp2, uint32_t dstOp) {
	Instruction* inst = new Instruction(instructionsList.size(), type, srcOp1, srcOp2, dstOp);
	instructionsList.push_back(inst);
}

bool CPU::isFinished() {
	bool finished = true;
	for(int i = 0; finished && i < instructionsList.size(); i++)
		finished &= instructionsList[i]->hasRetired();
	return finished;
}

void CPU::simulate() {
	hasProgress = true;
	while(!isFinished() && hasProgress) {
		hasProgress = false;
		tick();
		// Move on to the next cycle.
		cycle++;
	}
}

void CPU::tick() {
	// add physical registers that are freed in the previous cycle to freeList
	for(PhysicalRegister& pReg : freePhysRegsPrevCycle)
		freeList.addRegister(pReg);
	freePhysRegsPrevCycle.clear();
	// We process pipeline stages in opposite order to (try to) clear up
	// the subsequent stage before sending instruction forward from any
	// given stage.
	retire();
	complete();
	execute();
	//My fcns start
	issue();
	dispatch();
	decode();
	fetch();
	//My fcns end
	std::cerr << rob.toString() << "\n";
	std::cerr << "Reservation Stations : [\n";
	for(int i = 0; i < reservationStations.size(); i++) {
		std::cerr << "\t" << reservationStations[i]->toString() << "\n";
	}
	std::cerr << "]\n";
	std::cerr << mapTable.toString() << "\n";
	std::cerr << archMappingTable.toString() << "\n";
	std::cerr << freeList.toString() << "\n\n";
}

void CPU::fetch() {
	for(int i = 0; i < width && isFetching; i++) {
		// hasProgress should set if CPU has progress in any stage at each cycle
		if(fetchPtr >= instructionsList.size()) {
			isFetching = false;
			break;
		}
		bool res = decodeStage.push(instructionsList[fetchPtr]);
		// res is always true in this project
		if(res) {
			hasProgress = true;
			std::cerr << "Cycle #" << cycle << ": fetch   \t" << instructionsList[fetchPtr]->toString() << "\n";
			instructionsList[fetchPtr]->setFetchCycle(cycle);
			fetchPtr++;
		}
		else {
			break;
		}
		if(fetchPtr >= instructionsList.size()) {
			isFetching = false;
		}
	}
}

void CPU::decode() {
	for(int i = 0; i < width; i++) {
		if(decodeStage.isEmpty())
			break;
		Instruction* inst = decodeStage.front();
		bool res = dispatchStage.push(inst);
		// Should not remove the instruction if cannot be added to the next stage
		// The reason could be because of stalls in next stages
		// res is always true in this project
		if(res) {
			inst->setDecodeCycle(cycle);
			std::cerr << "Cycle #" << cycle << ": decode  \t" << inst->toString() << "\n";
			hasProgress = true;
			decodeStage.pop();
		}
		else
			break;
	}
}

void CPU::dispatch() {
	for(int i = 0; i < width; i++) {
		if(dispatchStage.isEmpty())
			break;
		// No free RoB Entry -> stall
		if(!rob.hasFreeEntry()) {
			break;
		}

		Instruction* inst = dispatchStage.front();

		// Check if corresponding RS is free
		RSType requiredType = inst->getReservationStation();
		uint32_t freeRSIndex = -1;
		for(int j = 0; j < reservationStations.size(); j++) {
			if(reservationStations[j]->getType() == requiredType &&
					reservationStations[j]->isBusy() == false) {
				freeRSIndex = j;
				break;
			}
		}
		// required RS is busy -> stall
		if(freeRSIndex == -1) {
			break;
		}

		// Renaming

		// No free register in the free list -> stall
		if(inst->getDstOp() != -1 && freeList.hasRegister() == false) {
			break;
		}
		std::string beforeRenaming = inst->toString();
		inst->setSrcPhysicalReg1(mapTable.getMapping(inst->getSrcOp1()));
		if(inst->getSrcOp2() != -1)
			inst->setSrcPhysicalReg2(mapTable.getMapping(inst->getSrcOp2()));
		PhysicalRegister T;		// By default, T = -1
		PhysicalRegister Told;	// By default, Told = -1
		if(inst->getDstOp() != -1) {
			T = freeList.popRegister();
			T.setReady(false);
			inst->setDstPhysicalReg(T);
			Told = mapTable.getMapping(inst->getDstOp());
			mapTable.setMapping(inst->getDstOp(), T);
		}
		else {
			inst->setDstPhysicalReg(T);
		}
		inst->setRenamed(true);

		// Add instruction to ROB
		rob.addInstruction(inst, T, Told);

		// Add instruction to Reservation Station
		reservationStations[freeRSIndex]->allocate(inst);
		// Instruction need the reservation as well to free it at execute stage
		inst->setAllocatedRs(reservationStations[freeRSIndex]);

		inst->setDispatchCycle(cycle);
		std::cerr << "Cycle #" << cycle << ": dispatch\t" << beforeRenaming << " ->\t" << inst->toString() << "\n";
		hasProgress = true;
		dispatchStage.pop();
	}
}

void CPU::issue() {
	// TODO Your code here
	// Going over all reservation stations and execute the ones that are ready
	int i;
	Instruction* inst;
	for(i = 0; i < width; i++){
		inst = reservationStations[i]->getInst();
		if(reservationStations[i]->isReadyToExecute()){
			executeStage.push(inst);
			// setIssueCycle for the instruction that is issued
			inst->setIssueCycle(cycle);
			hasProgress = true;
		}
	}
	
	// Uncomment and use the following two lines at the location which you issue an instruction
	// std::cerr << "Cycle #" << cycle << ": issue   \t" << [inst]->toString() << "\n";	// [inst] may need to be changed
	//hasProgress = true;

}

void CPU::execute() {
	// TODO Your code here
	// Uncomment and use the following two lines at the location which you execute an instruction
	// std::cerr << "Cycle #" << cycle << ": execute \t" << [inst]->toString() << "\n"; // [inst] may need to be changed
	// hasProgress = true;
	int i;
	int j;
	for(i = 0; i < width; i++) {
		if(executeStage.isEmpty())
			break;
		// setExecuteCycle for the instruction that is started its execution
		Instruction* inst = executeStage.front();
		inst->setExecuteCycle(cycle);
		// setExecTime of the instruction according to the execution time of RS
		inst->setExecTime(inst->getAllocatedRs()->getExecTime());
		// Free the reservation stations that are executed
		inst->getAllocatedRs()->free();
		// add executing instructions to completeStage
		inst->setCompleteCycle(cycle);	
		executeStage.pop();
		hasProgress = true;
	}
}

void CPU::complete() {
	// TODO Your code here
	std::vector<Instruction*>& instList = completeStage.getAllInstructions();
	int i;
	int j;
	int startExeCycle;
	int exeTime;
	for(i = 0; i < instList.size(); i++){
		// setCompleteCycle for the instruction that is completed
		Instruction* inst = instList[i];
		if(inst->getDstOp() != -1){
			instList.erase(instList.begin()+i);
			startExeCycle = inst->getExecuteCycle();
			exeTime = instList[i]->getExecTime();
			i--;
			if((startExeCycle + exeTime) <= cycle){
				// add instructions to retireStage that finished their execution time and current cycle
				retireStage.push(inst);
				
				// set ready bit of the destination register
				mapTable.setReadyBit(inst->getDstPhysicalReg().getRegNum());
				inst->getDstPhysicalReg().setReady(true);
				for(j = 0; j < reservationStations.size(); j++){
					// broadcast the result to mapping table and reservation stations
					reservationStations[j]->broadcastRegReady(inst->getDstPhysicalReg().getRegNum());	
				}
				
				hasProgress = true;
			}
		}
	}
	// Uncomment and use the following two lines at the location which you execute an instruction
	// std::cerr << "Cycle #" << cycle << ": complete\t" << [inst]->toString() << "\n"; // [inst] may need to be changed
	// hasProgress = true;
}

void CPU::retire() {
	// TODO Your code here
	if(rob.getHead() != NULL){
		Instruction* inst = rob.getHead()->getInst();
		// retire instructions from head of rob
		if(inst->hasCompleted()){
			// setRetireCycle for the instruction that is retired
			inst->setRetireCycle(cycle);
			
			// update freePhysRegsPrevCycle array that add the physical registers in current 
			//cycle to the free list in the beginning of next cycle
				//mapTable.setReadyBit(inst->getDstPhysicalReg().getRegNum());
			PhysicalRegister Told = rob.getHead()->getTold();
			freePhysRegsPrevCycle[cycle] = Told;
			// update architectural mapping table
			archMappingTable.setMapping(inst->getDstOp(), inst->getDstPhysicalReg());
			hasProgress = true;
		}
	}
	
	
	// Uncomment and use the following two lines at the location which you execute an instruction
	// std::cerr << "Cycle #" << cycle << ": retire  \t" << [inst]->toString() << "\n"; // [inst] may need to be changed
	// hasProgress = true;
}

void CPU::generateOutputFile(std::string outputFile) {
	std::ofstream out(outputFile);
	if(!out.is_open()) {
		std::cerr << "Cannot open output file to write!\n";
		exit(-1);
	}
	for(auto inst : instructionsList) {
		out << inst->getFetchCycle() << " " <<
				inst->getDecodeCycle() << " " <<
				inst->getDispatchCycle() << " " <<
				inst->getIssueCycle() << " " <<
				inst->getExecuteCycle() << " " <<
				inst->getCompleteCycle() << " " <<
				inst->getRetireCycle() << "\n";
	}
	out.close();
}

std::string CPU::toString() {
	std::stringstream str;
	str << "[OoO CPU cycle=" << cycle << "\n";
	return str.str();
}
