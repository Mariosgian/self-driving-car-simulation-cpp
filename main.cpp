#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <cstdlib>
#include <ctime>
#include <random>
#include <unordered_map>
#include <cmath>

using namespace std;

// Compile :
// g++ -std=c++17 -O2 -Wall main.cpp -o hw




// Προδήλωση γιατί το χρειαζομαι πιο πανω
class GridWorld;

//---- Basic types -----

struct Position {
    int x = 0;
    int y = 0;
};

enum class Direction {
    North,
    South,
    East,
    West
};

enum class TrafficLightColor {
    RED,
    YELLOW,
    GREEN
};

enum class CarSpeedState {
    STOPPED,
    HALF_SPEED,
    FULL_SPEED
};


// Global sensing context ώστε οι αισθητήρες να ξέρουν πού είναι το SDCAR
struct SensingContext {
    const GridWorld* world = nullptr;
    Position carPos;
    Direction carDir = Direction::North;
};

SensingContext g_sensingContext;


// Ρυθμίσεις προσομοίωσης
struct SimulationConfig {
    unsigned int seed = 0;   // 0 = current time
    int dimX = 40;
    int dimY = 40;
    int simulationTicks = 100;
    int numMovingCars = 3;
    int numMovingBikes = 4;
    int numParkedCars = 5;
    int numStopSigns = 2;
    int numTrafficLights = 2;
    double minConfidenceThreshold = 0.40; // 40%
};


// ----------------------
//  SENSOR READING 
// ----------------------

struct SensorReading {
    std::string objectId;
    std::string objectType;
    Position position;
    double distance = 0.0;
    double speed = 0.0;
    Direction direction = Direction::North;
    std::string signText;
    TrafficLightColor trafficLight = TrafficLightColor::RED;
    double confidence = 0.0;  // 0.0 - 1.0
};


// ----------------------
//  Sensors (declarations)
// ----------------------

class Sensor {
protected:
    std::string id;
public:
    explicit Sensor(const std::string& id_) : id(id_) {}
    virtual ~Sensor() = default;

    const std::string& getId() const { return id; }

    virtual std::vector<SensorReading> sense() = 0;
};

class LidarSensor : public Sensor {
public:
    static constexpr int RANGE = 9; // 9x9 γύρω από το όχημα

    LidarSensor() : Sensor("LIDAR:0") {}

    std::vector<SensorReading> sense() override;
};

class RadarSensor : public Sensor {
public:
    static constexpr int RANGE = 12; // 12 κελιά μπροστά

    RadarSensor() : Sensor("RADAR:0") {}

    std::vector<SensorReading> sense() override;
};

class CameraSensor : public Sensor {
public:
    static constexpr int RANGE = 7; // 7x7 μπροστά

    CameraSensor() : Sensor("CAMERA:0") {}

    std::vector<SensorReading> sense() override;
};



// ----------------------
//  SENSOR FUSION ENGINE 
// ----------------------
class SensorFusionEngine {
    double minConfidenceThreshold;

public:
    explicit SensorFusionEngine(double minConf)
        : minConfidenceThreshold(minConf) {}

    // allReadings: όλες οι μετρήσεις από όλους τους αισθητήρες
    std::vector<SensorReading> fuseSensorData(const std::vector<SensorReading>& allReadings) {
        // 1) Ομαδοποίηση ανά objectId
        std::unordered_map<std::string, std::vector<const SensorReading*>> grouped;
        for (const auto& r : allReadings) {
            // Αν δεν έχουμε κάποιο objectId, μπορούμε να το θεωρήσουμε "unknown"
            std::string key = r.objectId.empty() ? std::string("UNKNOWN") : r.objectId;
            grouped[key].push_back(&r);
        }

        std::vector<SensorReading> result;
        result.reserve(grouped.size());

        // 2) Για κάθε αντικείμενο, φτιάχνουμε μία fused μέτρηση
        for (const auto& [id, readings] : grouped) {
            if (readings.empty()) continue;

            bool hasBike = false;
            double sumW = 0.0;

            double sumX = 0.0;
            double sumY = 0.0;
            double sumDist = 0.0;
            double sumSpeed = 0.0;

            double maxConf = 0.0;

            // Θα κρατήσουμε "καλύτερα" direction / sign / color με βάση το μεγαλύτερο confidence
            double bestDirConf   = -1.0;
            double bestTextConf  = -1.0;
            double bestColorConf = -1.0;

            Direction bestDir = readings.front()->direction;
            std::string bestSignText = readings.front()->signText;
            TrafficLightColor bestColor = readings.front()->trafficLight;

            // Τύπος αντικειμένου (από την πιο σίγουρη μέτρηση)
            std::string fusedType = readings.front()->objectType;
            double bestTypeConf = -1.0;

            for (const auto* r : readings) {
                double w = std::max(0.0, r->confidence); // weight = confidence (>=0)
                sumW      += w;
                sumX      += r->position.x * w;
                sumY      += r->position.y * w;
                sumDist   += r->distance * w;
                sumSpeed  += r->speed * w;
                maxConf    = std::max(maxConf, r->confidence);

                if (r->objectType == "BIKE") {
                    hasBike = true;
                }

                // καλύτερος τύπος με βάση confidence
                if (!r->objectType.empty() && r->confidence > bestTypeConf) {
                    bestTypeConf = r->confidence;
                    fusedType = r->objectType;
                }

                // direction από την πιο σίγουρη μέτρηση
                if (r->confidence > bestDirConf) {
                    bestDirConf = r->confidence;
                    bestDir = r->direction;
                }

                // signText από την πιο σίγουρη μέτρηση που έχει κείμενο
                if (!r->signText.empty() && r->confidence > bestTextConf) {
                    bestTextConf = r->confidence;
                    bestSignText = r->signText;
                }

                // traffic light color από την πιο σίγουρη μέτρηση
                if (r->confidence > bestColorConf) {
                    bestColorConf = r->confidence;
                    bestColor = r->trafficLight;
                }
            }

            // 3) Έλεγχος κατωφλίου βεβαιότητας
            // - Αν maxConf < threshold ΚΑΙ ΔΕΝ είναι BIKE → πετάμε το object
            if (!hasBike && maxConf < minConfidenceThreshold) {
                continue;
            }

            SensorReading fused;
            fused.objectId   = id;
            fused.objectType = fusedType;

            if (sumW > 0.0) {
                fused.position.x = static_cast<int>(std::round(sumX / sumW));
                fused.position.y = static_cast<int>(std::round(sumY / sumW));
                fused.distance   = sumDist / sumW;
                fused.speed      = sumSpeed / sumW;
            } else {
                // Αν όλα είχαν 0 confidence (σπάνιο), πάρε την πρώτη μέτρηση όπως είναι
                fused.position   = readings.front()->position;
                fused.distance   = readings.front()->distance;
                fused.speed      = readings.front()->speed;
            }

            fused.direction    = bestDir;
            fused.signText     = bestSignText;
            fused.trafficLight = bestColor;
            fused.confidence   = maxConf;  // ή sumW / readings.size(), αλλά max είναι πιο "συντηρητικό"

            result.push_back(fused);
        }

        return result;
    }
};


// ----------------------
// All objects
// ----------------------

class WorldObject {
protected:
    std::string id;   // π.χ. "CAR:0", "LIGHT:1"
    char glyph;       // χαρακτήρας για visualization
    Position pos;

public:
    WorldObject(const std::string& id_, char glyph_, Position p)
        : id(id_), glyph(glyph_), pos(p) {}

    virtual ~WorldObject() = default;

    const std::string& getId() const { return id; }
    char getGlyph() const { return glyph; }
    Position getPosition() const { return pos; }
    void setPosition(Position p) { pos = p; }

    virtual void update() = 0;
};


class StaticObject : public WorldObject {
public:
    StaticObject(const std::string& id_, char glyph_, Position p)
        : WorldObject(id_, glyph_, p) {}

    void update() override {
        ;
    }
};


class StationaryVehicle : public StaticObject {
    static int counter;
public:
    StationaryVehicle(Position p)
        : StaticObject("PARKED:" + std::to_string(counter++), 'P', p) {
        std::cout << "[+PARKED: " << id << "] Parked at (" << p.x << ", " << p.y << ")\n";
    }

    ~StationaryVehicle() {
        std::cout << "[-PARKED: " << id << "] I'm being towed away!\n";
    }
};
int StationaryVehicle::counter = 0;



class TrafficSign : public StaticObject {
    static int counter;
    std::string text; // π.χ. "STOP"
public:
    TrafficSign(Position p, const std::string& signText)
        : StaticObject("SIGN:" + std::to_string(counter++), 'S', p),
          text(signText) {
        std::cout << "[+SIGN: " << id << "] Initialized at (" << p.x << ", " << p.y
                  << ") text=" << text << "\n";
    }

    ~TrafficSign() {
        std::cout << "[-SIGN: " << id << "] Removed.\n";
    }

    const std::string& getText() const { return text; }
};
int TrafficSign::counter = 0;



class TrafficLight : public StaticObject {
    static int counter;
    TrafficLightColor color;
    int tickCounter = 0;

    static char colorToGlyph(TrafficLightColor c) {
        switch (c) {
            case TrafficLightColor::RED:    return 'R';
            case TrafficLightColor::YELLOW: return 'Y';
            case TrafficLightColor::GREEN:  return 'G';
        }
        return 'R';
    }
public:
    TrafficLight(Position p, TrafficLightColor initial = TrafficLightColor::RED)
        : StaticObject("LIGHT:" + std::to_string(counter++),
                       colorToGlyph(initial),
                       p),
          color(initial) {
        std::cout << "[+LIGHT: " << id << "] Initialized at ("
                  << p.x << ", " << p.y << ") to " << glyph << "\n";
    }

    ~TrafficLight() {
        std::cout << "[-LIGHT: " << id << "] Turning off.\n";
    }

    void update() override {
        ++tickCounter;

        int totalCycle = 4 + 8 + 2; // 14 ticks
        int phase = tickCounter % totalCycle;

        if (phase < 4) {
            color = TrafficLightColor::RED;
        } else if (phase < 4 + 8) {
            color = TrafficLightColor::GREEN;
        } else {
            color = TrafficLightColor::YELLOW;
        }

        glyph = colorToGlyph(color);

        std::cout << "[LIGHT] tick=" << tickCounter
                  << " phase=" << phase
                  << " color=" << glyph << "\n";
    }

    TrafficLightColor getColor() const { return color; }
};
int TrafficLight::counter = 0;



class MovingObject : public WorldObject {
protected:
    Direction dir;
    int speed; // cells per tick

public:
    MovingObject(const std::string& id_, char glyph_, Position p,
                 Direction d, int speed_)
        : WorldObject(id_, glyph_, p), dir(d), speed(speed_) {}

    void update() override {
        switch (dir) {
            case Direction::North:  pos.y -= speed; break;
            case Direction::South:  pos.y += speed; break;
            case Direction::West:   pos.x -= speed; break;
            case Direction::East:   pos.x += speed; break;
        }
    }

    Direction getDirection() const { return dir; }
    int getSpeed() const { return speed; }
};


class MovingCar : public MovingObject {
    static int counter;
public:
    MovingCar(Position p, Direction d, int speed_)
        : MovingObject("CAR:" + std::to_string(counter++), 'C', p, d, speed_) {
        std::cout << "[+CAR: " << id << "] Initialized at ("
                  << p.x << ", " << p.y << ") facing " << static_cast<int>(d)
                  << " at " << speed << " units/tick\n";
    }

    ~MovingCar() {
        std::cout << "[-CAR: " << id << "] Our journey is complete!\n";
    }
};
int MovingCar::counter = 0;


// Κινούμενο ποδήλατο (B)
class MovingBike : public MovingObject {
    static int counter;
public:
    MovingBike(Position p, Direction d, int speed_)
        : MovingObject("BIKE:" + std::to_string(counter++), 'B', p, d, speed_) {
        std::cout << "[+BIKE: " << id << "] Created at ("
                  << p.x << ", " << p.y << ")\n";
    }

    ~MovingBike() {
        std::cout << "[-BIKE: " << id << "] Being locked away...\n";
    }
};
int MovingBike::counter = 0;



// ----------------------
//  NavigationSystem Class
// ----------------------

class NavigationSystem {
    SensorFusionEngine fusion;
    std::vector<Position> gpsTargets;
    std::size_t currentTargetIndex = 0;

public:
    NavigationSystem(double minConf = 0.40)
        : fusion(minConf) {}

    std::vector<SensorReading> process(const std::vector<SensorReading>& readings) {
        return fusion.fuseSensorData(readings);
    }

    void setTargets(const std::vector<Position>& targets) {
        gpsTargets = targets;
        currentTargetIndex = 0;
    }

    bool hasActiveTarget() const {
        return currentTargetIndex < gpsTargets.size();
    }

    Position getCurrentTarget() const {
        return gpsTargets[currentTargetIndex];
    }

    void advanceToNextTarget() {
        if (currentTargetIndex < gpsTargets.size()) {
            ++currentTargetIndex;
        }
    }
};


// ----------------------
//  Self-Driving Car class
// ----------------------

class SelfDrivingCar : public MovingObject {
    std::vector<std::unique_ptr<Sensor>> sensors;
    NavigationSystem nav;
    CarSpeedState speedState = CarSpeedState::STOPPED;

    void setSpeedState(CarSpeedState s) {
        speedState = s;
        switch (s) {
            case CarSpeedState::STOPPED:
                speed = 0;
                break;
            case CarSpeedState::HALF_SPEED:
                speed = 1;
                break;
            case CarSpeedState::FULL_SPEED:
                speed = 2;
                break;
        }
    }

    // Εφαρμογή κανόνων επιβράδυνσης βάσει fused sensor data
    void applySensorDecisions(const std::vector<SensorReading>& fused) {
        bool needStop = false;
        bool needSlow = false;

        for (const auto& r : fused) {
            int dx = r.position.x - pos.x;
            int dy = r.position.y - pos.y;

            bool inFront = false;
            switch (dir) {
                case Direction::North: inFront = (dy < 0 && std::abs(dy) <= 12 && std::abs(dx) <= 3); break;
                case Direction::South: inFront = (dy > 0 && std::abs(dy) <= 12 && std::abs(dx) <= 3); break;
                case Direction::West:  inFront = (dx < 0 && std::abs(dx) <= 12 && std::abs(dy) <= 3); break;
                case Direction::East:  inFront = (dx > 0 && std::abs(dx) <= 12 && std::abs(dy) <= 3); break;
            }

            if (!inFront) continue;

            // Φανάρια μπροστά εντός 3 κελιών
            if (r.objectType == "LIGHT" && r.distance <= 3.0) {
                if (r.trafficLight == TrafficLightColor::RED) {
                    needStop = true;
                } else if (r.trafficLight == TrafficLightColor::YELLOW) {
                    needSlow = true;
                }
            }

            // Εμπόδιο μπροστά (CAR/BIKE) εντός 2 κελιών
            if ((r.objectType == "CAR" || r.objectType == "BIKE") && r.distance <= 2.0) {
                needSlow = true;
            }
        }

        if (needStop) {
            setSpeedState(CarSpeedState::STOPPED);
        } else if (needSlow) {
            if (speedState == CarSpeedState::FULL_SPEED) {
                setSpeedState(CarSpeedState::HALF_SPEED);
            }
        }
    }

public:
    SelfDrivingCar(Position p,
                   Direction d,
                   double minConfidenceThreshold,
                   const std::vector<Position>& gpsTargets)
        : MovingObject("SDCAR:0", '@', p, d, 1),
          nav(minConfidenceThreshold)
    {
        sensors.push_back(std::make_unique<LidarSensor>());
        sensors.push_back(std::make_unique<RadarSensor>());
        sensors.push_back(std::make_unique<CameraSensor>());

        nav.setTargets(gpsTargets);
        setSpeedState(CarSpeedState::HALF_SPEED);

        std::cout << "[+NAV: SDCAR] Hello, I'll be your GPS today\n";
    }

    ~SelfDrivingCar() {
        std::cout << "[-NAV: SDCAR] You've arrived (or simulation ended). Shutting down...\n";
    }

    void update() override {
        // 1. Συλλογή δεδομένων αισθητήρων
        std::vector<SensorReading> all;
        for (auto& s : sensors) {
            auto readings = s->sense();
            all.insert(all.end(), readings.begin(), readings.end());
        }

        // 2. Συγχώνευση
        auto fused = nav.process(all);

        // 3. GPS λογική
        if (!nav.hasActiveTarget()) {
            setSpeedState(CarSpeedState::STOPPED);
            std::cout << "[SDCAR] No more GPS targets. Staying in place at ("
                      << pos.x << ", " << pos.y << ")\n";
            return;
        }

        Position target = nav.getCurrentTarget();
        Position here = pos;

        int dx = target.x - here.x;
        int dy = target.y - here.y;

        if (dx == 0 && dy == 0) {
            std::cout << "[SDCAR] Reached GPS target (" << target.x << ", " << target.y << ")\n";
            nav.advanceToNextTarget();

            if (!nav.hasActiveTarget()) {
                setSpeedState(CarSpeedState::STOPPED);
                std::cout << "[SDCAR] Reached final GPS target. Stopping.\n";
                return;
            }

            target = nav.getCurrentTarget();
            dx = target.x - here.x;
            dy = target.y - here.y;
        }

        // Κατεύθυνση προς τον στόχο (Manhattan: πρώτα οριζόντια, μετά κάθετα)
        if (dx != 0) {
            dir = (dx > 0) ? Direction::East : Direction::West;
        } else if (dy != 0) {
            dir = (dy > 0) ? Direction::South : Direction::North;
        }

        int manhattan = std::abs(dx) + std::abs(dy);

        // Κοντά στον στόχο GPS (<=5) → HALF_SPEED, αλλιώς FULL_SPEED
        if (manhattan <= 5) {
            setSpeedState(CarSpeedState::HALF_SPEED);
        } else {
            setSpeedState(CarSpeedState::FULL_SPEED);
        }

        // 4. Εφαρμογή κανόνων από αισθητήρες (φανάρια, εμπόδια)
        applySensorDecisions(fused);

        // 5. Κίνηση
        MovingObject::update();

        std::cout << "[SDCAR] Moving towards (" << target.x << ", " << target.y
                  << ") now at (" << pos.x << ", " << pos.y << "), dir="
                  << (dir == Direction::North ? "N" :
                      dir == Direction::South ? "S" :
                      dir == Direction::East  ? "E" : "W")
                  << ", speed=" << speed << "\n";
    }
};



// ----------------------
//  GridWorld Class
// ----------------------

class GridWorld {
    int width;
    int height;
    std::vector<std::unique_ptr<WorldObject>> objects;

public:
    GridWorld(int w, int h) : width(w), height(h) {
        std::cout << "[+WORLD] Reticulating splines - Hello, world!\n";
    }

    ~GridWorld() {
        std::cout << "[-WORLD] Goodbye, cruel world!\n";
    }

    const std::vector<std::unique_ptr<WorldObject>>& getObjects() const {
        return objects;
    }

    bool getSelfDrivingCarPosition(Position& out) const {
        for (const auto& obj : objects) {
            if (obj->getId() == "SDCAR:0") {
                out = obj->getPosition();
                return true;
            }
        }
        return false;
    }

    int getWidth() const { return width; }
    int getHeight() const { return height; }

    void addObject(std::unique_ptr<WorldObject> obj) {
        objects.push_back(std::move(obj));
    }

    void updateAll() {
        // Ενημέρωση sensing context για το SDCAR
        g_sensingContext.world = this;
        g_sensingContext.carPos = Position{0, 0};
        g_sensingContext.carDir = Direction::North;

        for (auto& obj : objects) {
            if (obj->getId() == "SDCAR:0") {
                g_sensingContext.carPos = obj->getPosition();
                if (auto* mo = dynamic_cast<MovingObject*>(obj.get())) {
                    g_sensingContext.carDir = mo->getDirection();
                }
                break;
            }
        }

        // Πρώτα όλοι κάνουν update
        for (auto& obj : objects) {
            obj->update();
        }

        // Μετά καθαρίζουμε όσα βγήκαν εκτός ορίων
        std::vector<std::unique_ptr<WorldObject>> remaining;
        remaining.reserve(objects.size());

        for (auto& obj : objects) {
            Position p = obj->getPosition();
            bool outOfBounds = (p.x < 0 || p.x >= width ||
                                p.y < 0 || p.y >= height);

            if (outOfBounds) {
                if (dynamic_cast<MovingObject*>(obj.get()) != nullptr) {
                    std::cout << "[WORLD] Removing moving object "
                              << obj->getId()
                              << " at (" << p.x << ", " << p.y
                              << ") - out of bounds.\n";
                } else {
                    remaining.push_back(std::move(obj));
                    continue;
                }
            } else {
                remaining.push_back(std::move(obj));
            }
        }

        objects.swap(remaining);
    }

    void printSummary() const {
        std::cout << "World " << width << "x" << height
                  << " has " << objects.size() << " objects.\n";
        for (const auto& obj : objects) {
            Position p = obj->getPosition();
            std::cout << "  " << obj->getId()
                      << " '" << obj->getGlyph()
                      << "' at (" << p.x << ", " << p.y << ")\n";
        }
    }
};


// ----------------------
// Visualization helpers
// ----------------------

int glyphPriority(char g) {
    switch (g) {
        case '@': return 0; // Self-Driving Car
        case 'R': return 1; // RED Light
        case 'Y': return 2; // YELLOW Light
        case 'S': return 3; // Stop Sign
        case 'B': return 4; // Moving Bike
        case 'C': return 5; // Moving Car
        case 'G': return 6; // GREEN Light
        case 'P': return 7; // Parked Car
        case '?': return 8; // Unknown
        case '.': return 9; // Empty cell
        default:  return 8; // Ό,τι άλλο σαν unknown
    }
}

char glyphAtCell(const GridWorld& world, int x, int y) {
    char bestGlyph = '.';
    int bestPriority = glyphPriority('.');

    const auto& objs = world.getObjects();
    for (const auto& obj : objs) {
        Position p = obj->getPosition();
        if (p.x == x && p.y == y) {
            char g = obj->getGlyph();
            int pr = glyphPriority(g);
            if (pr < bestPriority) {
                bestPriority = pr;
                bestGlyph = g;
            }
        }
    }

    return bestGlyph;
}

void visualization_pov_centered(const GridWorld& world, const Position& carPos, int radius) {
    int w = world.getWidth();
    int h = world.getHeight();

    std::cout << "=== POV (centered) around SDCAR at ("
              << carPos.x << ", " << carPos.y << "), radius=" << radius << " ===\n";

    for (int dy = -radius; dy <= radius; ++dy) {
        for (int dx = -radius; dx <= radius; ++dx) {
            int x = carPos.x + dx;
            int y = carPos.y + dy;

            if (x < 0 || x >= w || y < 0 || y >= h) {
                std::cout << 'X';
            } else {
                std::cout << glyphAtCell(world, x, y);
            }
        }
        std::cout << '\n';
    }

    std::cout << "===============================================\n";
}

void visualization_full(const GridWorld& world) {
    int w = world.getWidth();
    int h = world.getHeight();

    std::cout << "=== FULL WORLD VIEW ===\n";
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            std::cout << glyphAtCell(world, x, y);
        }
        std::cout << '\n';
    }
    std::cout << "=======================\n";
}



// ----------------------
// Sensors::sense implementations
// ----------------------

static std::string typeFromId(const std::string& id) {
    auto pos = id.find(':');
    if (pos == std::string::npos) return "UNKNOWN";
    return id.substr(0, pos);
}

std::vector<SensorReading> LidarSensor::sense() {
    std::vector<SensorReading> readings;
    if (!g_sensingContext.world) return readings;

    const auto& objs = g_sensingContext.world->getObjects();
    Position carPos = g_sensingContext.carPos;

    // 9x9 γύρω από το όχημα => |dx|,|dy| <= 4
    for (const auto& ptr : objs) {
        auto* obj = ptr.get();
        if (obj->getId() == "SDCAR:0") continue;

        Position p = obj->getPosition();
        int dx = p.x - carPos.x;
        int dy = p.y - carPos.y;

        if (std::abs(dx) > 4 || std::abs(dy) > 4) continue;

        SensorReading r;
        r.objectId = obj->getId();
        r.objectType = typeFromId(r.objectId);
        r.position = p;
        r.distance = std::abs(dx) + std::abs(dy);
        r.speed = 0.0;
        r.direction = Direction::North;
        r.signText = "";
        r.trafficLight = TrafficLightColor::RED;

        double base = 0.99; // εξαιρετική ακρίβεια
        double rangeFactor = 1.0 - (r.distance / static_cast<double>(RANGE));
        if (rangeFactor < 0.0) rangeFactor = 0.0;
        r.confidence = base * rangeFactor;

        readings.push_back(r);
    }

    return readings;
}

std::vector<SensorReading> RadarSensor::sense() {
    std::vector<SensorReading> readings;
    if (!g_sensingContext.world) return readings;

    const auto& objs = g_sensingContext.world->getObjects();
    Position carPos = g_sensingContext.carPos;
    Direction dir = g_sensingContext.carDir;

    for (const auto& ptr : objs) {
        auto* obj = ptr.get();
        if (obj->getId() == "SDCAR:0") continue;

        std::string type = typeFromId(obj->getId());
        if (type != "CAR" && type != "BIKE") continue; // μόνο κινούμενα

        Position p = obj->getPosition();
        int dx = p.x - carPos.x;
        int dy = p.y - carPos.y;

        int dist = 0;
        bool inBeam = false;

        switch (dir) {
            case Direction::East:
                if (dy == 0 && dx > 0 && dx <= RANGE) { dist = dx; inBeam = true; }
                break;
            case Direction::West:
                if (dy == 0 && dx < 0 && -dx <= RANGE) { dist = -dx; inBeam = true; }
                break;
            case Direction::South:
                if (dx == 0 && dy > 0 && dy <= RANGE) { dist = dy; inBeam = true; }
                break;
            case Direction::North:
                if (dx == 0 && dy < 0 && -dy <= RANGE) { dist = -dy; inBeam = true; }
                break;
        }

        if (!inBeam) continue;

        SensorReading r;
        r.objectId = obj->getId();
        r.objectType = type;
        r.position = p;
        r.distance = dist;

        if (auto* mob = dynamic_cast<MovingObject*>(obj)) {
            r.speed = mob->getSpeed();
            r.direction = mob->getDirection();
        } else {
            r.speed = 0.0;
            r.direction = Direction::North;
        }

        r.signText = "";
        r.trafficLight = TrafficLightColor::RED;

        double base = 0.99; // υψηλή ακρίβεια
        if (type == "BIKE") base = 0.95;
        double rangeFactor = 1.0 - (dist / static_cast<double>(RANGE));
        if (rangeFactor < 0.0) rangeFactor = 0.0;
        r.confidence = base * rangeFactor;

        readings.push_back(r);
    }

    return readings;
}

std::vector<SensorReading> CameraSensor::sense() {
    std::vector<SensorReading> readings;
    if (!g_sensingContext.world) return readings;

    const auto& objs = g_sensingContext.world->getObjects();
    Position carPos = g_sensingContext.carPos;
    Direction dir = g_sensingContext.carDir;

    for (const auto& ptr : objs) {
        auto* obj = ptr.get();
        if (obj->getId() == "SDCAR:0") continue;

        Position p = obj->getPosition();
        int dx = p.x - carPos.x;
        int dy = p.y - carPos.y;

        bool inFov = false;

        // 7x7 μπροστά: λωρίδα πλάτους 7 (±3) και μήκους 7 μπροστά
        switch (dir) {
            case Direction::East:
                inFov = (dx > 0 && dx <= RANGE && std::abs(dy) <= 3);
                break;
            case Direction::West:
                inFov = (dx < 0 && -dx <= RANGE && std::abs(dy) <= 3);
                break;
            case Direction::South:
                inFov = (dy > 0 && dy <= RANGE && std::abs(dx) <= 3);
                break;
            case Direction::North:
                inFov = (dy < 0 && -dy <= RANGE && std::abs(dx) <= 3);
                break;
        }

        if (!inFov) continue;

        SensorReading r;
        r.objectId = obj->getId();
        r.objectType = typeFromId(r.objectId);
        r.position = p;
        r.distance = std::abs(dx) + std::abs(dy);

        if (auto* mob = dynamic_cast<MovingObject*>(obj)) {
            r.speed = mob->getSpeed();
            r.direction = mob->getDirection();
        } else {
            r.speed = 0.0;
            r.direction = Direction::North;
        }

        r.signText = "";
        if (auto* sign = dynamic_cast<TrafficSign*>(obj)) {
            r.signText = sign->getText();
        }

        r.trafficLight = TrafficLightColor::RED;
        if (auto* light = dynamic_cast<TrafficLight*>(obj)) {
            r.trafficLight = light->getColor();
        }

        double base = 0.95; // υψηλή για κατηγοριοποίηση
        double rangeFactor = 1.0 - (r.distance / static_cast<double>(RANGE));
        if (rangeFactor < 0.0) rangeFactor = 0.0;
        r.confidence = base * rangeFactor;

        readings.push_back(r);
    }

    return readings;
}



// ----------------------
// printHelp
// ----------------------

void printHelp() {
    std::cout << "Self-Driving Car Simulation" << std::endl;
    std::cout << "Usage:" << std::endl;
    std::cout << "  --seed <n>                  Random seed (default: current time)" << std::endl;
    std::cout << "  --dimX <n>                  World width (default: 40)" << std::endl;
    std::cout << "  --dimY <n>                  World height (default: 40)" << std::endl;
    std::cout << "  --numMovingCars <n>         Number of moving cars (default: 3)" << std::endl;
    std::cout << "  --numMovingBikes <n>        Number of moving bikes (default: 4)" << std::endl;
    std::cout << "  --numParkedCars <n>         Number of parked cars (default: 5)" << std::endl;
    std::cout << "  --numStopSigns <n>          Number of stop signs (default: 2)" << std::endl;
    std::cout << "  --numTrafficLights <n>      Number of traffic lights (default: 2)" << std::endl;
    std::cout << "  --simulationTicks <n>       Maximum simulation ticks (default: 100)" << std::endl;
    std::cout << "  --minConfidenceThreshold <n> Minimum confidence cut off (default: 40)" << std::endl;
    std::cout << "  --gps <x1> <y1> [x2 y2 ...] GPS target coordinates (required)" << std::endl;
    std::cout << "  --help                      Show this help message" << std::endl << std::endl;

    std::cout << "Example usage:" << std::endl;
    std::cout << "  ./oop_proj_2025 --seed 12 --dimY 50 --gps 10 20 32 15" << std::endl;
}



// ------ MAIN -------

int main(int argc, char* argv[]) {
    SimulationConfig config;
    std::vector<Position> gpsTargets;

    if (argc == 1) {
        std::cout << "No arguments provided. Use --help for usage.\n";
        return 0;
    }

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "--help") {
            printHelp();
            return 0;
        }
        else if (arg == "--seed") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --seed\n";
                return 1;
            }
            config.seed = static_cast<unsigned int>(std::stoul(argv[++i]));
        }
        else if (arg == "--dimX") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --dimX\n";
                return 1;
            }
            config.dimX = std::stoi(argv[++i]);
        }
        else if (arg == "--dimY") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --dimY\n";
                return 1;
            }
            config.dimY = std::stoi(argv[++i]);
        }
        else if (arg == "--simulationTicks") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --simulationTicks\n";
                return 1;
            }
            config.simulationTicks = std::stoi(argv[++i]);
        }
        else if (arg == "--numMovingCars") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --numMovingCars\n";
                return 1;
            }
            config.numMovingCars = std::stoi(argv[++i]);
        }
        else if (arg == "--numMovingBikes") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --numMovingBikes\n";
                return 1;
            }
            config.numMovingBikes = std::stoi(argv[++i]);
        }
        else if (arg == "--numParkedCars") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --numParkedCars\n";
                return 1;
            }
            config.numParkedCars = std::stoi(argv[++i]);
        }
        else if (arg == "--numStopSigns") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --numStopSigns\n";
                return 1;
            }
            config.numStopSigns = std::stoi(argv[++i]);
        }
        else if (arg == "--numTrafficLights") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --numTrafficLights\n";
                return 1;
            }
            config.numTrafficLights = std::stoi(argv[++i]);
        }
        else if (arg == "--minConfidenceThreshold") {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for --minConfidenceThreshold\n";
                return 1;
            }
            int percent = std::stoi(argv[++i]);
            config.minConfidenceThreshold = percent / 100.0;
        }
        else if (arg == "--gps") {
            if (i + 2 >= argc) {
                std::cerr << "You must provide at least one GPS pair after --gps\n";
                return 1;
            }

            ++i;
            while (i + 1 < argc) {
                int x = std::stoi(argv[i]);
                int y = std::stoi(argv[i + 1]);
                gpsTargets.push_back(Position{x, y});
                i += 2;
            }
            break;
        }
        else {
            std::cerr << "Unknown parameter: " << arg << "\n";
            std::cerr << "Use --help to see valid parameters.\n";
            return 1;
        }
    }

    if (gpsTargets.empty()) {
        std::cerr << "No GPS targets provided. Use --gps x y [x2 y2 ...]\n";
        return 1;
    }

    GridWorld world(config.dimX, config.dimY);

    // Random generator με βάση το seed
    unsigned int seed = config.seed;
    if (seed == 0) {
        seed = std::random_device{}();
    }
    std::mt19937 rng(seed);

    std::uniform_int_distribution<int> distX(0, config.dimX - 1);
    std::uniform_int_distribution<int> distY(0, config.dimY - 1);
    auto randomPos = [&]() {
        return Position{distX(rng), distY(rng)};
    };

    std::uniform_int_distribution<int> distDir(0, 3);
    auto randomDir = [&]() {
        int d = distDir(rng);
        switch (d) {
            case 0: return Direction::North;
            case 1: return Direction::South;
            case 2: return Direction::West;
            case 3: return Direction::East;
        }
        return Direction::North;
    };

    // Στατικά αντικείμενα
    for (int i = 0; i < config.numParkedCars; ++i) {
        world.addObject(std::make_unique<StationaryVehicle>(randomPos()));
    }

    for (int i = 0; i < config.numStopSigns; ++i) {
        world.addObject(std::make_unique<TrafficSign>(randomPos(), "STOP"));
    }

    for (int i = 0; i < config.numTrafficLights; ++i) {
        world.addObject(std::make_unique<TrafficLight>(randomPos(), TrafficLightColor::RED));
    }

    // Κινούμενα αντικείμενα
    for (int i = 0; i < config.numMovingCars; ++i) {
        world.addObject(std::make_unique<MovingCar>(randomPos(), randomDir(), 1));
    }

    for (int i = 0; i < config.numMovingBikes; ++i) {
        world.addObject(std::make_unique<MovingBike>(randomPos(), randomDir(), 1));
    }

    // Self-driving car
    world.addObject(std::make_unique<SelfDrivingCar>(
        Position{0, 0},
        Direction::North,
        config.minConfidenceThreshold,
        gpsTargets
    ));

    std::cout << "Initial world created ("
              << config.dimX << "x" << config.dimY << ").\n";
    world.printSummary();

    visualization_full(world);

    std::cout << "Starting simulation for " << config.simulationTicks << " ticks...\n";

    for (int tick = 0; tick < config.simulationTicks; ++tick) {
        std::cout << "=== Tick " << tick << " ===\n";
        world.updateAll();

        Position carPos;
        if (world.getSelfDrivingCarPosition(carPos)) {
            visualization_pov_centered(world, carPos, 5);
        } else {
            std::cout << "[POV] Self-driving car not found in world.\n";
        }
    }

    std::cout << "Simulation finished.\n";
    world.printSummary();
    visualization_full(world);

    return 0;
}
